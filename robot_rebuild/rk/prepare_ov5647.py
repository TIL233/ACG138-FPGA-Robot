"""Configure the supplied ACG138 CSI->ISP route; optionally supervise its AIQ server.

Default is read-only discovery. No kernel, firmware, serial or motor operations.
Based on ACG138_board_files/tests/board/camera_isp_service.sh, single-camera path.
"""
from pathlib import Path
import argparse
import json
import os
import signal
import subprocess
import threading
import time

from camera_capture import discover_csi, run_command, load_config, isp_capture_size


def configure_route(route, run=run_command, *,width=1296,height=972):
    cif, isp = route['cif'], route['isp']
    out_width,out_height=isp_capture_size(width,height)
    left,top=(width-out_width)//2,(height-out_height)//2
    run(['media-ctl', '-d', cif, '-V', f'"{route["sensor"]}":0 [fmt:SBGGR10_1X10/{width}x{height}]'])
    configure_raw(route, run,width=width,height=height)
    run(['media-ctl', '-d', isp, '-V', f'"rkisp-isp-subdev":0 [fmt:SBGGR10_1X10/{width}x{height} crop:({left},{top})/{out_width}x{out_height}]'])
    run(['media-ctl', '-d', isp, '-V', f'"rkisp-isp-subdev":2 [fmt:YUYV8_2X8/{out_width}x{out_height} crop:(0,0)/{out_width}x{out_height}]'])
    run(['v4l2-ctl', '-d', route['video'], f'--set-fmt-video=width={out_width},height={out_height},pixelformat=NV12'])
    run(['v4l2-ctl', '-d', route['video'], f'--set-selection=target=crop,left=0,top=0,width={out_width},height={out_height}'])


def configure_raw(route, run=run_command, *,width=1296,height=972):
    # Reassert compact RAW after AIQ prepare, as required by the supplied BSP.
    fmt = ['v4l2-ctl', '-d', route['raw'], f'--set-fmt-video=width={width},height={height},pixelformat=BG10']
    run(fmt)
    run(['v4l2-ctl', '-d', route['raw'], f'--set-selection=target=crop,left=0,top=0,width={width},height={height}'])
    run(fmt)


def existing_aiq(proc=Path('/proc')):
    found = []
    for entry in proc.iterdir():
        if not entry.name.isdigit(): continue
        try:
            args = (entry/'cmdline').read_bytes().split(b'\0')
            if args and Path(os.fsdecode(args[0])).name in ('rkaiq_3A_server', 'rkaiq-online-server', 'rkaiq-single-camera'):
                found.append(int(entry.name))
        except (OSError, ValueError): pass
    return found


def install_iq(source, index, directory=Path('/etc/iqfiles')):
    # Only install the known BSP IQ name; never overwrite an existing calibration.
    data = Path(source).read_bytes()
    parsed = json.loads(data)
    if not isinstance(parsed, dict) or not parsed:
        raise ValueError('IQ source must be a nonempty JSON object from the matching BSP')
    destination = directory/f'ov5647_ACG138-OV5647-CSI{index}_default.json'
    directory.mkdir(parents=True, exist_ok=True)
    try:
        with destination.open('xb') as stream: stream.write(data)
    except FileExistsError:
        if destination.read_bytes() != data:
            raise RuntimeError('Existing IQ differs; keep its calibration and inspect manually: '+str(destination))
    return destination


def aiq_environment(route, run=None, *,width=1296,height=972):
    run=run or run_command
    device=run(['media-ctl','-d',route['isp'],'-e','rkisp-input-params'])
    import re
    if not re.fullmatch(r'/dev/video\d+',device):
        raise ValueError('Invalid ISP params device discovered')
    return dict(os.environ, ACG138_INDEPENDENT_ISPS='1', normal_no_read_back='0',
                ACG138_SENSOR=route['sensor'],ACG138_ISP_PARAMS=device,
                ACG138_WIDTH=str(width),ACG138_HEIGHT=str(height))


def configure_raw_route(route, *,width,height):
    import fcntl,array
    run_command(['media-ctl','-d',route['cif'],'-V',f'"{route["sensor"]}":0 [fmt:SBGGR10_1X10/{width}x{height}]'])
    with open(route['raw'],'r+b',buffering=0) as stream:
        # RKCIF_CMD_SET_CSI_MEMORY_MODE, from the supplied kernel UAPI.
        request=0x40000000 | (4<<16) | (ord('V')<<8) | (192+1)
        fcntl.ioctl(stream,request,array.array('i',[0]))
    configure_raw(route,width=width,height=height)


def serve_raw(route,runtime, *,width,height):
    import fcntl
    runtime.mkdir(parents=True,exist_ok=True)
    with (runtime/'owner.lock').open('a') as owner:
        fcntl.flock(owner,fcntl.LOCK_EX|fcntl.LOCK_NB)
        ready=runtime/'ready.json';ready.unlink(missing_ok=True)
        found=existing_aiq()
        if found:raise RuntimeError('Another AIQ server still owns the camera: '+repr(found))
        stopped=threading.Event()
        previous={sig:signal.signal(sig,lambda *_:stopped.set()) for sig in (signal.SIGTERM,signal.SIGINT)}
        try:
            configure_raw_route(route,width=width,height=height)
            state=dict(index=route['index'],width=width,height=height,backend='raw',parent_pid=os.getpid(),aiq_pid=None)
            pending=runtime/'ready.json.pending';pending.write_text(json.dumps(state));pending.replace(ready)
            print(json.dumps(dict(route,raw_prepared=True,video_verified=False)),flush=True)
            while not stopped.wait(.5):pass
        finally:
            ready.unlink(missing_ok=True)
            for sig,handler in previous.items():signal.signal(sig,handler)


def serve(route, executable, iq_source, runtime, *,width=1296,height=972):
    import fcntl
    executable = Path(executable).resolve(strict=True)
    if not os.access(executable, os.X_OK):
        raise ValueError('AIQ server must be an executable from the matching board BSP')
    runtime.mkdir(parents=True, exist_ok=True)
    with (runtime/'owner.lock').open('a') as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        ready=runtime/'ready.json'
        ready.unlink(missing_ok=True)
        found = existing_aiq()
        if found:
            raise RuntimeError('Another AIQ server is running: '+repr(found)+'; stop its owning service before starting this one')
        install_iq(iq_source, route['index'])
        configure_route(route,width=width,height=height)
        stopped = threading.Event()
        previous = {sig: signal.signal(sig, lambda *_: stopped.set()) for sig in (signal.SIGTERM, signal.SIGINT)}
        process = None
        try:
            env = aiq_environment(route,width=width,height=height)
            with (runtime/'aiq.log').open('wb') as log:
                process = subprocess.Popen([str(executable)], stdout=log, stderr=subprocess.STDOUT, env=env)
                deadline = time.monotonic()+20
                while not stopped.is_set():
                    if process.poll() is not None:
                        raise RuntimeError('AIQ exited; inspect '+str(runtime/'aiq.log'))
                    if b'wait stream start event' in (runtime/'aiq.log').read_bytes(): break
                    if time.monotonic() >= deadline:
                        raise TimeoutError('AIQ prepare timeout; inspect '+str(runtime/'aiq.log'))
                    stopped.wait(.2)
                if stopped.is_set(): return
                # In readback mode AIQ has allocated/owns the CIF buffers and
                # selects their layout. Reasserting S_FMT here returns EBUSY
                # or conflicts with that layout. Only the online BSP path
                # requires reasserting compact capture after preparation.
                if env['normal_no_read_back']=='1':
                    configure_raw(route,width=width,height=height)
                state=dict(index=route['index'],width=width,height=height,backend='isp',
                           parent_pid=os.getpid(),aiq_pid=process.pid)
                pending=runtime/'ready.json.pending'
                pending.write_text(json.dumps(state))
                pending.replace(ready)
                print(json.dumps(dict(route, aiq_prepared=True, video_verified=False)), flush=True)
                # Prepared does not mean frames have arrived. Viewer reports live frames.
                while not stopped.wait(.5):
                    if process.poll() is not None: raise RuntimeError('AIQ server exited')
        finally:
            ready.unlink(missing_ok=True)
            if process is not None and process.poll() is None:
                process.terminate()
                try: process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
            for sig, handler in previous.items(): signal.signal(sig, handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index', type=int, choices=(0, 1), default=None)
    parser.add_argument('--camera-config',type=Path,default=Path(__file__).with_name('camera_config.json'))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--apply', action='store_true', help='configure graph only; requires an independently managed matching AIQ service')
    mode.add_argument('--serve', action='store_true', help='configure graph and supervise BSP AIQ server in foreground')
    parser.add_argument('--aiq-server', default='/userdata/acg138-camera/rkaiq-online-server')
    parser.add_argument('--iq-source', default='/userdata/acg138-camera/iq/ov5647-isp30.json')
    parser.add_argument('--runtime', type=Path, default=Path('/run/robot-ov5647'))
    args = parser.parse_args(argv)
    config=load_config(args.camera_config)
    route = discover_csi(config.csi_index if args.index is None else args.index)
    if args.serve:
        if config.csi_backend=='raw':serve_raw(route,args.runtime,width=config.csi_width,height=config.csi_height)
        else:serve(route, args.aiq_server, args.iq_source, args.runtime,width=config.csi_width,height=config.csi_height)
    else:
        if args.apply:
            if config.csi_backend=='raw':configure_raw_route(route,width=config.csi_width,height=config.csi_height)
            else:configure_route(route,width=config.csi_width,height=config.csi_height)
        print(json.dumps(dict(route, configured=args.apply, video_verified=False), ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
