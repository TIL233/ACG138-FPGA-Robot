"""Headless, read-only robot monitor; reuse a single camera and detector."""
import json
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2
import numpy as np
from hdmi_camera import Viewer, ROOT, SERIAL, WIDTH, HEIGHT, motion_status, concise_status
from protocol import PosixTelemetryLink
from camera_capture import load_config,ov5647_only
from video_recording import VideoRecorder,RemoteRecordTrigger,load_recording_config,recording_presentation

PAGE = r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>小车实时查看</title><style>body{background:#121a24;color:#eee;font:18px sans-serif;margin:24px}main{max-width:1100px;margin:auto}img{width:100%;background:#000;min-height:180px}pre{white-space:pre-wrap;line-height:1.6}small{color:#abc}</style>
<main><h2>小车实时查看</h2><small>只读 · 人/瓶子识别 · 不发送电机指令</small><p><img id="video" alt="等待摄像头画面"></p><pre id="status">正在连接…</pre></main>
<script>
const names=['停止','前进','后退','普通左转','普通右转','定点左转','定点右转'];
async function tick(){try{const r=await fetch('/status',{cache:'no-store'});if(!r.ok)throw Error('连接失败');const s=await r.json(),t=s.telemetry;
let a=[s.camera_live?'OV5647在线':'OV5647无画面：'+s.camera_error,s.recognition_live?`识别：人 ${s.people}  瓶子 ${s.bottles}  推理 ${s.inference_ms.toFixed(0)} ms`:'识别：'+s.recognition_error];
if(s.recording_notice)a.push(s.recording_notice.text);
if(t){const distance=(v,b)=>(t.health&b)&&v!==65535?(v/10).toFixed(1)+' cm':'无有效数据';a.push(`距离 左 ${distance(t.distance_l,4)}  右 ${distance(t.distance_r,8)}`,`轮速 左 ${t.velocity_l}  右 ${t.velocity_r} 脉冲/20ms`,t.health&2?`横滚 ${(t.roll/100).toFixed(2)}°  俯仰 ${(t.pitch/100).toFixed(2)}°  航向 ${(t.yaw/100).toFixed(2)}°`:'姿态无有效数据',`控制请求 ${names[t.command]||'未知'}  速度档 ${t.speed}  状态 ${t.state}  故障 0x${t.fault.toString(16).padStart(4,'0')}`)}else a.push('FPGA遥测无数据：'+s.serial_error);
a.push(s.motion);document.querySelector('#status').textContent=a.join('\n');const v=document.querySelector('#video');v.style.visibility=s.camera_live?'visible':'hidden';if(s.camera_live)v.src='/frame.jpg?t='+Date.now();
}catch(e){document.querySelector('#status').textContent='连接中断，请检查网线和RK3588电源';document.querySelector('#video').style.visibility='hidden'}setTimeout(tick,250)}tick();
</script></html>'''.encode()


class Monitor(Viewer):
    def __init__(self):
        self.camera_config=ov5647_only(load_config(ROOT/'camera_config.json'))
        self.camera_name='OV5647'
        self.recorder=VideoRecorder(load_recording_config(ROOT/'recording_config.json'),WIDTH,HEIGHT)
        self.remote_record=RemoteRecordTrigger()
        self.link = PosixTelemetryLink(SERIAL)
        self.camera_frames = 0
        self.camera_lock = threading.Lock()
        self.camera_latest = (None,0.,0,'等待摄像头')
        self.recognition = (None,0.,0,[],0.,'模型加载中')
        self.stop_event = threading.Event()
        self.process = None
        self.log = open(ROOT/'camera-pipeline.log','ab')
        self.snapshot_lock = threading.Lock()
        self.snapshot = ({},b'',0.)
        self.threads = [threading.Thread(target=f,daemon=True) for f in (self.read_camera,self.recognize,self.publish)]
        for thread in self.threads: thread.start()

    def publish(self):
        last = None
        jpeg = b''
        while not self.stop_event.is_set():
            now = time.monotonic()
            self.link.poll(now)
            self.remote_record_step(now)
            with self.camera_lock:
                data, stamp, frame, error = self.camera_latest
                rgb, ds, df, objects, ms, de = self.recognition
            camera_live = data is not None and now-stamp < 1
            detection_live = camera_live and rgb is not None and now-ds < 1
            key = (detection_live,df if detection_live else frame)
            if camera_live and key != last:
                image=np.frombuffer(rgb if detection_live else data,np.uint8).reshape(HEIGHT,WIDTH,3)
                ok, encoded=cv2.imencode('.jpg',image[:,:,::-1],[cv2.IMWRITE_JPEG_QUALITY,75])
                if ok: jpeg=encoded.tobytes();last=key
            live=self.link.live(now)
            t=self.link.telemetry if live else None
            record_lines=concise_status(camera_name='OV5647',source='usb',view_live=camera_live,
                telemetry=t,live=live,objects=objects,detection_live=detection_live,
                recording=self.recorder.snapshot())
            self.offer_recording((rgb if detection_live else data) if camera_live else None,
                ds if detection_live else stamp,record_lines,now)
            status=dict(camera_kind='ov5647',recording=self.recorder.snapshot(),camera_live=camera_live,camera_frames=frame,camera_error=error,
                telemetry_live=live,telemetry=vars(t).copy() if t else None,serial_error=self.link.error,
                recognition_live=detection_live,recognition_error=de or '等待有效识别结果',inference_ms=ms,
                people=sum(o[0]==0 for o in objects) if detection_live else 0,
                bottles=sum(o[0]==39 for o in objects) if detection_live else 0,motion=motion_status(t))
            status['recording_notice']=recording_presentation(status['recording'])
            with self.snapshot_lock:self.snapshot=(status,jpeg if camera_live else b'',now)
            self.stop_event.wait(.05)


def main():
    monitor=Monitor()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            path=self.path.split('?',1)[0]
            with monitor.snapshot_lock:status,jpeg,stamp=monitor.snapshot
            fresh=time.monotonic()-stamp<1
            if path=='/':body,kind,code=PAGE,'text/html; charset=utf-8',200
            elif path=='/status' and fresh:body,kind,code=json.dumps(status,ensure_ascii=False).encode(),'application/json',200
            elif path=='/frame.jpg' and fresh and jpeg:body,kind,code=jpeg,'image/jpeg',200
            else:body,kind,code=b'Not available','text/plain',503
            self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store');self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
    server=ThreadingHTTPServer(('169.254.100.100',8080),Handler)
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        server.server_close();monitor.stop_event.set()
        for thread in monitor.threads:thread.join(3)
        try:monitor.recorder.close()
        finally:monitor.link.close();monitor.log.close()


if __name__=='__main__':main()
