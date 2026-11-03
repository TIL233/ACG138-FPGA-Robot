"""OV5647 CIF compact RAW10 -> framed RGB on stdout; no motor/serial access.

Rockchip compact RAW10 stores four consecutive 10-bit little-endian samples in
five bytes, with a per-line stride reported by the driver. It is not the MIPI
wire packing. Preview, recognition and recording consume this same RGB stream.
"""
import argparse
import os
import re
import select
import signal
import subprocess
import sys
import time
import cv2
import numpy as np


def unpack_raw10(data,width,height,stride):
    if width<=0 or height<=0 or width%4 or stride<width*5//4 or len(data)!=stride*height:
        raise ValueError('Invalid compact RAW10 frame or row stride')
    rows=np.frombuffer(data,np.uint8).reshape(height,stride)
    g=rows[:,:width*5//4].reshape(height,width//4,5)
    result=np.empty((height,width//4,4),np.uint8)
    result[:,:,0]=(g[:,:,0]>>2)|((g[:,:,1]&3)<<6)
    result[:,:,1]=(g[:,:,1]>>4)|((g[:,:,2]&15)<<4)
    result[:,:,2]=(g[:,:,2]>>6)|((g[:,:,3]&63)<<2)
    result[:,:,3]=g[:,:,4]
    return result.reshape(height,width)


def fit_rgb(rgb,width,height,rotation=0):
    if rotation:
        rgb=cv2.rotate(rgb,{90:cv2.ROTATE_90_CLOCKWISE,180:cv2.ROTATE_180,270:cv2.ROTATE_90_COUNTERCLOCKWISE}[rotation])
    h,w=rgb.shape[:2];scale=min(width/w,height/h)
    target_w=max(1,min(width,round(w*scale)));target_h=max(1,min(height,round(h*scale)))
    scaled=cv2.resize(rgb,(target_w,target_h),interpolation=cv2.INTER_AREA if scale<1 else cv2.INTER_LINEAR)
    result=np.zeros((height,width,3),np.uint8)
    x,y=(width-target_w)//2,(height-target_h)//2
    result[y:y+target_h,x:x+target_w]=scaled
    return result


def next_exposure(exposure,gain,level,exposure_limits,gain_limits):
    ratio=48./max(1.,level-16.)
    if .9<=ratio<=1.1:return exposure,gain
    ratio=max(.8,min(1.25,ratio))
    lo,hi=exposure_limits;glo,ghi=gain_limits
    proposed=max(lo,min(hi,round(exposure*ratio)))
    if (ratio>1 and proposed==hi) or (ratio<1 and gain>glo):
        gain=max(glo,min(ghi,round(gain*ratio)))
    return proposed,gain


def control_ranges(sensor):
    p=subprocess.run(['v4l2-ctl','-d',sensor,'--list-ctrls'],capture_output=True,text=True,timeout=3)
    if p.returncode:raise RuntimeError('Cannot read OV5647 controls: '+p.stderr)
    result={}
    for name in ('exposure','analogue_gain','vertical_blanking'):
        line=next((line for line in p.stdout.splitlines() if re.match(r'\s*'+name+r'\s',line)),None)
        if line is None:raise RuntimeError('Missing OV5647 control '+name)
        values={k:int(v) for k,v in re.findall(r'(min|max|default|value)=(-?\d+)',line)}
        if not all(k in values for k in ('min','max','default')):raise RuntimeError('Incomplete control '+name)
        result[name]=values
    return result


def set_controls(sensor,values):
    p=subprocess.run(['v4l2-ctl','--silent','-d',sensor,'--set-ctrl',','.join(f'{k}={v}' for k,v in values.items())],capture_output=True,text=True,timeout=2)
    if p.returncode:raise RuntimeError('OV5647 exposure control failed: '+p.stderr)


def main(argv=None):
    cv2.setNumThreads(2)
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device',required=True);parser.add_argument('--sensor-device',required=True)
    parser.add_argument('--csi-index',type=int,choices=(0,1),default=0)
    parser.add_argument('--width',type=int,required=True);parser.add_argument('--height',type=int,required=True)
    parser.add_argument('--output-width',type=int,default=1280);parser.add_argument('--output-height',type=int,default=720)
    parser.add_argument('--rotation',type=int,choices=(0,90,180,270),default=0)
    args=parser.parse_args(argv)
    from camera_capture import CameraConfig,require_isp_ready
    owner_config=CameraConfig(csi_index=args.csi_index,csi_width=args.width,csi_height=args.height,csi_backend='raw')
    require_isp_ready(owner_config)
    if (not re.fullmatch(r'/dev/video\d+',args.device) or not re.fullmatch(r'/dev/v4l-subdev\d+',args.sensor_device)
            or (args.width,args.height) not in ((640,480),(1296,972),(1920,1080),(2592,1944))
            or not 1<=args.output_width<=1920 or not 1<=args.output_height<=1080):
        parser.error('Invalid OV5647 device or dimensions')
    env=dict(os.environ,LC_ALL='C')
    p=subprocess.run(['v4l2-ctl','-d',args.device,'--get-fmt-video'],capture_output=True,text=True,timeout=3,env=env)
    stride=re.search(r'Bytes per Line\s*:\s*(\d+)',p.stdout)
    shape=re.search(r'Width/Height\s*:\s*(\d+)/(\d+)',p.stdout)
    if p.returncode or not stride or not shape or tuple(map(int,shape.groups()))!=(args.width,args.height) or "'BG10'" not in p.stdout:
        raise RuntimeError('RAW node is not configured for the requested OV5647 mode')
    stride=int(stride[1]);size=stride*args.height
    image_size=re.search(r'Size Image\s*:\s*(\d+)',p.stdout)
    if not image_size or int(image_size[1])!=size:raise RuntimeError('Unsupported RAW buffer padding')
    limits=control_ranges(args.sensor_device)
    blank=limits['vertical_blanking'];set_controls(args.sensor_device,{'vertical_blanking':blank['default']})
    limits=control_ranges(args.sensor_device)
    el=(limits['exposure']['min'],min(limits['exposure']['max'],args.height+blank['default']-4))
    gl=(limits['analogue_gain']['min'],min(limits['analogue_gain']['max'],128))
    exposure=max(el[0],min(el[1],450));gain=max(gl[0],min(gl[1],32))
    set_controls(args.sensor_device,dict(exposure=exposure,analogue_gain=gain))
    stopped=False
    def stop(*_):
        nonlocal stopped
        stopped=True
    previous={sig:signal.signal(sig,stop) for sig in (signal.SIGTERM,signal.SIGINT)}
    process=None
    try:
        process=subprocess.Popen(['v4l2-ctl','--silent','-d',args.device,'--stream-mmap=4','--stream-count=0','--stream-to=-','--stream-poll'],stdout=subprocess.PIPE,bufsize=0,env=env)
        pending=bytearray();last_frame=time.monotonic();next_output=last_frame;period=1/15;count=0;next_owner_check=last_frame
        gains=np.ones(3);lut=np.repeat(np.arange(256,dtype=np.uint8)[:,None,None],3,axis=2)
        while not stopped:
            if time.monotonic()>=next_owner_check:
                require_isp_ready(owner_config);next_owner_check=time.monotonic()+.25
            if time.monotonic()-last_frame>3:raise TimeoutError('OV5647 RAW capture timed out')
            ready,_,_=select.select([process.stdout],[],[],.2)
            if not ready:continue
            data=os.read(process.stdout.fileno(),size-len(pending))
            if not data:raise RuntimeError('OV5647 RAW capture disconnected')
            pending.extend(data)
            if len(pending)!=size:continue
            stamp=time.monotonic();last_frame=stamp
            if stamp<next_output:
                pending.clear();continue
            next_output+=(int((stamp-next_output)/period)+1)*period
            bayer=unpack_raw10(pending,args.width,args.height,stride)
            rgb=cv2.cvtColor(bayer,cv2.COLOR_BayerBGGR2RGB);pending.clear()
            means=np.array(cv2.mean(rgb)[:3]);count+=1
            if count==1 or count%10==0:
                neutral=np.maximum(1.,means-16.);wanted=np.clip(neutral.mean()/neutral,.65,2.)
                gains=.85*gains+.15*wanted
                linear=np.maximum(0.,np.arange(256,dtype=float)-16.)[:,None]*gains[None,:]/239.
                lut=np.clip(np.sqrt(linear)*255.,0,255).astype(np.uint8).reshape(256,1,3)
                new_exposure,new_gain=next_exposure(exposure,gain,float(means.mean()),el,gl)
                if (new_exposure,new_gain)!=(exposure,gain):
                    set_controls(args.sensor_device,dict(exposure=new_exposure,analogue_gain=new_gain))
                    exposure,gain=new_exposure,new_gain
            rgb=cv2.LUT(rgb,lut)
            output=fit_rgb(rgb,args.output_width,args.output_height,args.rotation)
            sys.stdout.buffer.write(output.tobytes());sys.stdout.buffer.flush()
    finally:
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=2)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=2)
            process.stdout.close()
        for sig,handler in previous.items():signal.signal(sig,handler)
    return 0


if __name__=='__main__':
    try:raise SystemExit(main())
    except (BrokenPipeError,KeyboardInterrupt):pass
    except Exception as exc:
        print('OV5647 RAW: '+str(exc),file=sys.stderr);raise SystemExit(1)
