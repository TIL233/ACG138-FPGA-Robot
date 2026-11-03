"""Deploy the configurable camera viewer with backup and checksum verification."""
from pathlib import Path
import argparse
from datetime import datetime
import hashlib
import json
import shlex
import uuid
import sys
from dataclasses import asdict
from board_access import connect

ROOT = Path(__file__).resolve().parent
REMOTE = '/home/ubuntu/fpga-hdmi-viewer'
sys.path.insert(0,str(ROOT.parent/'robot_rebuild/rk'))
from camera_capture import CameraConfig,ov5647_only
from video_recording import RecordingConfig
from usb_lidar import UsbLidarConfig


def run(client, command):
    _, out, err = client.exec_command(command, timeout=30)
    stdout, stderr = out.read().decode(errors='replace'), err.read().decode(errors='replace')
    code = out.channel.recv_exit_status()
    if code:
        raise RuntimeError(f'Remote command failed ({code}): {stderr or stdout}')
    return stdout, stderr


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera-config', metavar='JSON', help='explicit camera configuration; otherwise preserve board camera_config.json')
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--follow',action='store_true',help='persist person-follow UI; remote 0 explicitly starts')
    modes.add_argument('--navigate',metavar='ROUTE_JSON',help='persist navigation UI; never automatically starts motion')
    modes.add_argument('--mission',metavar='MISSION_JSON',help='persist semantic navigation/person/bottle task UI; explicit start required')
    modes.add_argument('--read-only',action='store_true',help='explicitly replace startup with the receive-only viewer; otherwise preserve existing startup mode')
    args=parser.parse_args()
    preserve_startup=not(args.follow or args.navigate or args.mission or args.read_only)
    camera_data = Path(args.camera_config).read_bytes() if args.camera_config else (ROOT.parent/'robot_rebuild/rk/camera_config.json').read_bytes()
    camera_data=json.dumps(asdict(ov5647_only(CameraConfig(**json.loads(camera_data)))),ensure_ascii=False,indent=2).encode('utf8')
    route_data=None
    mission_data=None
    if args.navigate:
        sys.path.insert(0,str(ROOT.parent/'robot_rebuild/rk'))
        from navigation import load_route
        load_route(args.navigate)
        route_data=Path(args.navigate).read_bytes()
    if args.mission:
        sys.path.insert(0,str(ROOT.parent/'robot_rebuild/rk'))
        from mission import MissionSession,load_mission
        MissionSession(**load_mission(args.mission))
        mission_data=Path(args.mission).read_bytes()
    client = connect()
    sftp = None
    deploy = 'sync-' + datetime.now().strftime('%Y%m%d') + '-' + uuid.uuid4().hex[:8]
    stage, backup = REMOTE + '/' + deploy, REMOTE + '/backup-' + deploy
    scripts = {
        'hdmi_camera.py': (ROOT/'hdmi_camera.py').read_bytes(),
        'hdmi_telemetry.py': (ROOT/'hdmi_telemetry.py').read_bytes(),
        'browser_viewer.py': (ROOT/'browser_viewer.py').read_bytes(),
        'protocol.py': (ROOT.parent/'robot_rebuild/rk/protocol.py').read_bytes(),
        'lidar_planning.py': (ROOT.parent/'robot_rebuild/rk/lidar_planning.py').read_bytes(),
        'lidar_map_view.py': (ROOT.parent/'robot_rebuild/rk/lidar_map_view.py').read_bytes(),
        'usb_lidar.py': (ROOT.parent/'robot_rebuild/rk/usb_lidar.py').read_bytes(),
        'usb_lidar_config.json': (ROOT.parent/'robot_rebuild/rk/usb_lidar_config.json').read_bytes(),
        'detector.py': (ROOT.parent/'robot_rebuild/rk/detector.py').read_bytes(),
        'csi_capture.py': (ROOT.parent/'robot_rebuild/rk/csi_capture.py').read_bytes(),
        'ov5647_raw_capture.py': (ROOT.parent/'robot_rebuild/rk/ov5647_raw_capture.py').read_bytes(),
        'video.py': (ROOT.parent/'robot_rebuild/rk/video.py').read_bytes(),
        'camera_capture.py': (ROOT.parent/'robot_rebuild/rk/camera_capture.py').read_bytes(),
        'video_recording.py': (ROOT.parent/'robot_rebuild/rk/video_recording.py').read_bytes(),
        'recording_overlay.py': (ROOT.parent/'robot_rebuild/rk/recording_overlay.py').read_bytes(),
        'recording_config.json': (ROOT.parent/'robot_rebuild/rk/recording_config.json').read_bytes(),
        'prepare_ov5647.py': (ROOT.parent/'robot_rebuild/rk/prepare_ov5647.py').read_bytes(),
        'rkaiq_single_camera.c': (ROOT.parent/'robot_rebuild/rk/rkaiq_single_camera.c').read_bytes(),
        'robot-ov5647.service': (ROOT.parent/'robot_rebuild/rk/robot-ov5647.service').read_bytes(),
        'camera_config.json': camera_data,
        'follow.py': (ROOT.parent/'robot_rebuild/rk/follow.py').read_bytes(),
        'navigation.py': (ROOT.parent/'robot_rebuild/rk/navigation.py').read_bytes(),
        'mission.py': (ROOT.parent/'robot_rebuild/rk/mission.py').read_bytes(),
        'mission_example.json': (ROOT.parent/'robot_rebuild/rk/mission_example.json').read_bytes(),
        'mission_semantic_example.json': (ROOT.parent/'robot_rebuild/rk/mission_semantic_example.json').read_bytes(),
        'mission_bottle_example.json': (ROOT.parent/'robot_rebuild/rk/mission_bottle_example.json').read_bytes(),
        'route_example.json': (ROOT.parent/'robot_rebuild/rk/route_example.json').read_bytes(),
        'yolov8n.onnx': (ROOT.parent/'robot_rebuild/rk/models/yolov8n.onnx').read_bytes(),
        'start-camera-viewer.sh': b'#!/bin/sh\nexport DISPLAY="${DISPLAY:-:0}"\nexport XAUTHORITY="${XAUTHORITY:-/home/ubuntu/.Xauthority}"\nexport NO_AT_BRIDGE=1\nexport PYTHONPATH=/home/ubuntu/fpga-hdmi-viewer/runtime\nxset s noblank\nxset s 0 0\nxset -dpms\ncd /home/ubuntu/fpga-hdmi-viewer || exit 1\nexec /usr/bin/python3 -B hdmi_camera.py >>camera-viewer.log 2>&1\n',
    }
    if args.follow:
        scripts['start-camera-viewer.sh']=scripts['start-camera-viewer.sh'].replace(
            b'-B hdmi_camera.py >>',b'-B hdmi_camera.py --follow person >>')
    if route_data is not None:
        scripts['route.json']=route_data
        scripts['start-camera-viewer.sh']=scripts['start-camera-viewer.sh'].replace(
            b'-B hdmi_camera.py >>',b'-B hdmi_camera.py --navigate route.json >>')
    if mission_data is not None:
        scripts['mission.json']=mission_data
        scripts['start-camera-viewer.sh']=scripts['start-camera-viewer.sh'].replace(
            b'-B hdmi_camera.py >>',b'-B hdmi_camera.py --mission mission.json >>')
    report = {'deployment': deploy, 'backup': backup, 'files': {},'preserved_configuration':[], 'removed_files':[]}
    try:
        sftp = client.open_sftp()
        sftp.mkdir(stage)
        sftp.mkdir(backup)
        for name, data in scripts.items():
            try:
                with sftp.file(REMOTE+'/'+name, 'rb') as f:
                    previous = f.read()
            except FileNotFoundError:
                previous = None
            if previous is not None:
                with sftp.file(backup+'/'+name, 'wb') as f:
                    f.write(previous)
                if name in ('mission_semantic_example.json','route_example.json','recording_config.json','usb_lidar_config.json') or (name=='camera_config.json' and not args.camera_config):
                    # This file may now contain the user's calibrated places.
                    # Updating code must not silently replace their coordinates.
                    data=previous
                    report['preserved_configuration'].append(name)
                if name=='start-camera-viewer.sh' and preserve_startup:
                    # A code-only update must not silently remove a configured
                    # follow/navigation UI or enable writing in a read-only UI.
                    # Preserve the complete local launcher, including options.
                    data=previous
                    report['preserved_configuration'].append(name)
            if name=='camera_config.json':
                previous_config=CameraConfig(**json.loads(data))
                migrated=ov5647_only(previous_config)
                if migrated!=previous_config:data=json.dumps(asdict(migrated),ensure_ascii=False,indent=2).encode('utf8')
            if name=='recording_config.json':RecordingConfig(**json.loads(data))
            if name=='usb_lidar_config.json':UsbLidarConfig(**json.loads(data))
            with sftp.file(stage+'/'+name, 'wb') as f:
                f.write(data)
            with sftp.file(stage+'/'+name, 'rb') as f:
                actual = f.read()
            expected = hashlib.sha256(data).hexdigest()
            if hashlib.sha256(actual).hexdigest() != expected:
                raise RuntimeError('Upload checksum mismatch: '+name)
            report['files'][name] = expected
        run(client, 'python3 -m py_compile '+ ' '.join(shlex.quote(stage+'/'+n) for n in scripts if n.endswith('.py')))
        # Stop only exact viewer script tokens, never a broad shell pattern.
        stop = '''import os,signal,time
from pathlib import Path
targets={'/home/ubuntu/fpga-hdmi-viewer/hdmi_camera.py','/home/ubuntu/fpga-hdmi-viewer/hdmi_telemetry.py','/home/ubuntu/fpga-hdmi-viewer/browser_viewer.py'}
pids=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  args=(p/'cmdline').read_bytes().split(b'\\0')
  absolute=any(a.decode(errors='replace') in targets for a in args)
  relative=any(a in (b'hdmi_camera.py',b'hdmi_telemetry.py',b'browser_viewer.py') for a in args) and (p/'cwd').resolve()==Path('/home/ubuntu/fpga-hdmi-viewer')
  if absolute or relative:pids.append(int(p.name))
 except (OSError,ProcessLookupError):pass
for pid in pids:
 try:os.kill(pid,signal.SIGTERM)
 except ProcessLookupError:pass
deadline=time.monotonic()+5
while any(Path('/proc',str(pid)).exists() for pid in pids) and time.monotonic()<deadline:time.sleep(.1)
if any(Path('/proc',str(pid)).exists() for pid in pids):raise RuntimeError('Viewer did not exit; files not replaced')
print('Stopped viewer PIDs:',pids)
'''
        print(run(client,'python3 -c '+shlex.quote(stop))[0].strip())
        for name in scripts:
            sftp.posix_rename(stage+'/'+name, REMOTE+'/'+name)
        for name in ('uvc_capture.py',):
            try:
                with sftp.file(REMOTE+'/'+name,'rb') as f:old_camera=f.read()
            except FileNotFoundError:continue
            with sftp.file(backup+'/'+name,'wb') as f:f.write(old_camera)
            sftp.remove(REMOTE+'/'+name)
            report['removed_files'].append(name)
        sftp.chmod(REMOTE+'/start-camera-viewer.sh',0o755)
        autostart = '/home/ubuntu/.config/autostart/robot-usb-camera.desktop'
        try:
            with sftp.file(autostart,'rb') as f:
                old = f.read()
            with sftp.file(backup+'/robot-usb-camera.desktop','wb') as f:
                f.write(old)
        except FileNotFoundError:
            pass
        entry='[Desktop Entry]\nType=Application\nName=Robot OV5647 Camera Display\nExec=/home/ubuntu/fpga-hdmi-viewer/start-camera-viewer.sh\nTerminal=false\nOnlyShowIn=XFCE;\n'
        with sftp.file(autostart+'.pending','w') as f:
            f.write(entry)
        sftp.posix_rename(autostart+'.pending',autostart)
        report['autostart'] = autostart
        run(client,'nohup '+shlex.quote(REMOTE+'/start-camera-viewer.sh')+' </dev/null >/dev/null 2>&1 &')
        report['started'] = True
        (ROOT/'hdmi-sync-result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(report,ensure_ascii=False,indent=2))
    finally:
        if sftp is not None:sftp.close()
        client.close()


if __name__=='__main__':
    main()
