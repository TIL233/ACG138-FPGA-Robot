"""ACG138 OV5647 ISP capture. Legacy configuration fields remain readable."""
from dataclasses import dataclass, replace
from pathlib import Path
import json
import os
import re
import select
import subprocess
import sys
import threading
import time

USB_DEVICE = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera-video-index0'


@dataclass(frozen=True)
class CameraConfig:
    primary: str = 'ov5647'
    usb_device: str = USB_DEVICE
    csi_index: int = 0
    rotation: int = 0
    preview: str = 'off'
    fpga_monitor: bool = False
    csi_width: int = 1296
    csi_height: int = 972
    csi_backend: str = 'raw'

    def __post_init__(self):
        if type(self.fpga_monitor) is not bool:
            raise ValueError('fpga_monitor must be true or false')
        if self.primary not in ('usb', 'ov5647', 'fpga'):
            raise ValueError('primary must be usb, ov5647 or fpga')
        if type(self.csi_index) is not int or self.csi_index not in (0, 1):
            raise ValueError('csi_index must be 0 (FPC2) or 1 (FPC1)')
        if (type(self.csi_width) is not int or type(self.csi_height) is not int
                or (self.csi_width,self.csi_height) not in ((640,480),(1296,972),(1920,1080),(2592,1944))):
            raise ValueError('Unsupported native OV5647 CSI size')
        if self.csi_backend not in ('raw','isp'):raise ValueError('csi_backend must be raw or isp')
        if type(self.rotation) is not int or self.rotation not in (0, 90, 180, 270):
            raise ValueError('rotation must be 0, 90, 180 or 270')
        if self.preview not in ('ov5647', 'off'):
            raise ValueError('preview must be ov5647 or off')
        if not isinstance(self.usb_device, str) or not self.usb_device.startswith('/dev/') or any(c.isspace() for c in self.usb_device):
            raise ValueError('usb_device must be an absolute /dev device path without whitespace')


def load_config(path, missing_ok=True):
    path = Path(path)
    if not path.exists():
        if missing_ok: return CameraConfig()
        raise FileNotFoundError(path)
    data = json.loads(path.read_text(encoding='utf-8-sig'))
    if not isinstance(data, dict):
        raise ValueError('camera config must be a JSON object')
    return CameraConfig(**data)


def ov5647_only(config):
    """Migrate operational camera selection while preserving CSI/rotation."""
    return replace(config,primary='ov5647',preview='off',fpga_monitor=False)


def isp_capture_size(width,height):
    # The binned sensor mode is 1296x972. ISP/GStreamer size ranges require
    # aligned endpoints; crop 8/6 pixels per side to standard 1280x960.
    return (1280,960) if (width,height)==(1296,972) else (width,height)


def require_isp_ready(config, *,runtime=None,proc_root=Path('/proc')):
    # Windows tests never probe board runtime. Explicit paths allow testing the
    # gate without a real service or process; Linux capture always requires it.
    if runtime is None and os.name!='posix':return
    runtime=Path('/run/robot-ov5647') if runtime is None else Path(runtime)
    try:
        state=json.loads((runtime/'ready.json').read_text())
        backend=state.get('backend','isp')
        if backend!=config.csi_backend:raise ValueError('camera backend differs from prepared service')
        if (state['index'],state['width'],state['height'])!=(config.csi_index,config.csi_width,config.csi_height):
            raise ValueError('camera mode differs from prepared ISP')
        owners=[('parent_pid','prepare_ov5647.py')]
        if backend=='isp':owners.append(('aiq_pid','rkaiq-single-camera'))
        for key,expected in owners:
            pid=state[key]
            if type(pid) is not int or pid<=0:raise ValueError('invalid ISP owner')
            args=(proc_root/str(pid)/'cmdline').read_bytes().split(b'\0')
            if not any(Path(os.fsdecode(a)).name==expected for a in args if a):
                raise ValueError('ISP owner no longer running')
    except (OSError,KeyError,TypeError,ValueError) as exc:
        raise RuntimeError('OV5647 ISP not ready; waiting for camera service: '+str(exc)) from exc


def run_command(argv):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=8)
    except FileNotFoundError as exc:
        raise RuntimeError('Missing camera tool: '+argv[0]+' (install v4l-utils / GStreamer)') from exc
    if result.returncode:
        raise RuntimeError('Camera command failed: '+repr(argv)+': '+result.stderr.strip())
    return result.stdout.strip()


def find_media(platform, run=run_command, devices=None):
    devices = sorted(Path('/dev').glob('media*')) if devices is None else devices
    matches = []
    for device in devices:
        try:
            topology = run(['media-ctl', '-d', str(device), '-p'])
        except RuntimeError:
            continue
        if re.search(r'^bus info\s+platform:'+re.escape(platform)+r'\s*$', topology, re.M):
            matches.append((str(device), topology))
    if len(matches) != 1:
        raise RuntimeError(f'Expected one {platform} media graph, found {len(matches)}; check OV5647 kernel/device tree and CSI cable')
    return matches[0]


def discover_csi(index=0, run=run_command, devices=None):
    if type(index) is not int or index not in (0, 1):
        raise ValueError('Only CSI0/FPC2 and CSI1/FPC1 are supported')
    cif, topology = find_media('rkcif-mipi-lvds'+('1' if index else ''), run, devices)
    sensors = re.findall(r'entity\s+\d+: (m0'+str(index)+r'_b_ov5647 [^\n]+?) \(', topology)
    if len(sensors) != 1:
        raise RuntimeError(f'OV5647 CSI{index} sensor not found in {cif}; check probe, module and cable')
    isp, _ = find_media(f'rkisp{index}-vir0', run, devices)
    video = run(['media-ctl', '-d', isp, '-e', 'rkisp_mainpath'])
    raw = run(['media-ctl', '-d', cif, '-e', 'stream_cif_mipi_id0'])
    if not re.fullmatch(r'/dev/video\d+', video) or not re.fullmatch(r'/dev/video\d+', raw):
        raise RuntimeError('media-ctl returned an invalid capture node')
    return dict(index=index, cif=cif, isp=isp, sensor=sensors[0], video=video, raw=raw)


def capture_command(config, kind=None, width=1280, height=720, run=run_command):
    kind = config.primary if kind is None else kind
    if kind == 'ov5647':
        require_isp_ready(config)
        route = discover_csi(config.csi_index, run)
        if config.csi_backend=='raw':
            sensor_device=run(['media-ctl','-d',route['cif'],'-e',route['sensor']])
            if not re.fullmatch(r'/dev/v4l-subdev\d+',sensor_device):raise RuntimeError('Invalid OV5647 sensor device')
            return [sys.executable,'-B',str(Path(__file__).with_name('ov5647_raw_capture.py')),
                    '--device',route['raw'],'--sensor-device',sensor_device,
                    '--csi-index',str(config.csi_index),
                    '--width',str(config.csi_width),'--height',str(config.csi_height),
                    '--output-width',str(width),'--output-height',str(height),'--rotation',str(config.rotation)]
        source_width,source_height=isp_capture_size(config.csi_width,config.csi_height)
        source = ['v4l2src', f'device={route["video"]}', 'io-mode=mmap', '!',
                  f'video/x-raw,format=NV12,width={source_width},height={source_height}']
        if config.rotation:
            source += ['!', 'videoflip', 'method='+{90:'clockwise',180:'rotate-180',270:'counterclockwise'}[config.rotation]]
    else:
        raise ValueError('Only OV5647 ISP capture is enabled; migrate the camera configuration')
    return ['gst-launch-1.0', '-q'] + source + ['!', 'videoconvert', '!', 'videoscale',
        'add-borders=true', '!', f'video/x-raw,format=RGB,width={width},height={height},pixel-aspect-ratio=1/1',
        '!', 'queue', 'max-size-buffers=1', 'max-size-bytes=0', 'max-size-time=0', 'leaky=downstream',
        '!', 'fdsink', 'fd=1', 'sync=false']


class CaptureReader:
    """OV5647 RGB worker with bounded read, retry and shutdown."""
    def __init__(self, config, kind='ov5647', width=1280, height=720):
        self.config, self.kind = config, kind
        self.width, self.height = width, height
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = (None, 0., 0, '')
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        count = 0
        while not self.stop_event.is_set():
            process = None
            try:
                def checked_run(argv):
                    if self.stop_event.is_set(): raise InterruptedError('closing capture')
                    return run_command(argv)
                command = capture_command(self.config, self.kind, self.width, self.height, checked_run)
                if self.stop_event.is_set(): break
                # Inherit stderr rather than create an unread pipe that can deadlock.
                process = subprocess.Popen(command, stdout=subprocess.PIPE, bufsize=0)
                pending = bytearray()
                last_frame = time.monotonic()
                size = self.width*self.height*3
                while not self.stop_event.is_set():
                    if time.monotonic()-last_frame > 3:
                        raise TimeoutError('OV5647 capture timed out; check CSI/ISP service')
                    ready, _, _ = select.select([process.stdout], [], [], .2)
                    if not ready:
                        continue
                    chunk = os.read(process.stdout.fileno(), size-len(pending))
                    if not chunk:
                        raise OSError('OV5647 capture exited/disconnected; see camera-viewer.log')
                    pending.extend(chunk)
                    if len(pending) == size:
                        count += 1
                        last_frame = time.monotonic()
                        with self.lock:
                            self.latest = (bytes(pending), last_frame, count, '')
                        pending.clear()
            except Exception as exc:
                if not self.stop_event.is_set():
                    with self.lock:
                        self.latest = (None, 0., count, str(exc))
            finally:
                if process is not None:
                    if process.poll() is None:
                        process.terminate()
                        try: process.wait(timeout=1)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait(timeout=1)
                    process.stdout.close()
            self.stop_event.wait(2)

    def snapshot(self):
        with self.lock:
            return self.latest

    def close(self):
        self.stop_event.set()
        self.thread.join(12)  # A media-ctl probe is individually bounded to 8 s.
        if self.thread.is_alive():
            raise TimeoutError('CSI preview worker did not stop')
