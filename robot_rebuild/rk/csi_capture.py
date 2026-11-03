"""Standalone detector child protocol: capture stamp + 640x480 BGR from OV5647."""
from pathlib import Path
import signal,struct,sys,time
import numpy as np
from camera_capture import CaptureReader,load_config,ov5647_only


def main(argv=None):
    argv=sys.argv[1:] if argv is None else argv
    config=ov5647_only(load_config(Path(argv[0]) if argv else Path(__file__).with_name('camera_config.json'),missing_ok=False))
    reader=CaptureReader(config,width=640,height=480)
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    previous=None
    try:
        while True:
            rgb,stamp,count,error=reader.snapshot()
            if rgb is not None and count!=previous and 0<=time.monotonic()-stamp<1:
                bgr=np.frombuffer(rgb,np.uint8).reshape(480,640,3)[:,:,::-1].tobytes()
                sys.stdout.buffer.write(struct.pack('<d',stamp)+bgr);sys.stdout.buffer.flush();previous=count
            time.sleep(.01)
    except (KeyboardInterrupt,BrokenPipeError):pass
    finally:reader.close()


if __name__=='__main__':main()
