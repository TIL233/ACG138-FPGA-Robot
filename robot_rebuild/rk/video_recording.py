"""OV5647 recording from shared RGB frames; write only to a mounted USB volume.

No camera ownership, serial writes, mounting, deletion of old recordings, or
fallback to the system disk. Linux directory descriptors pin the USB target.
"""
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
import json,math,os,re,subprocess,threading,time,uuid
from recording_overlay import composite_rgb


@dataclass(frozen=True)
class RecordingConfig:
    fps: int = 10
    segment_seconds: int = 300
    minimum_free_mb: int = 512
    max_segment_mb: int = 256
    usb_mount: str = ''

    def __post_init__(self):
        for name,low,high in [('fps',1,30),('segment_seconds',1,1800),
                              ('minimum_free_mb',1,65536),('max_segment_mb',1,1024)]:
            value=getattr(self,name)
            if type(value) is not int or not low<=value<=high:
                raise ValueError(f'{name} must be {low}..{high}')
        if not isinstance(self.usb_mount,str) or (self.usb_mount and not PurePosixPath(self.usb_mount).is_absolute()):
            raise ValueError('usb_mount must be empty or an absolute mounted USB path')


def load_recording_config(path):
    path=Path(path)
    return RecordingConfig(**json.loads(path.read_text(encoding='utf-8-sig'))) if path.exists() else RecordingConfig()


def unescape_mount(text):
    return re.sub(r'\\([0-7]{3})',lambda m:chr(int(m[1],8)),text)


def mount_table(text=None):
    if text is None:text=Path('/proc/self/mountinfo').read_text(encoding='utf8')
    found={}
    for line in text.splitlines():
        fields=line.split()
        try:
            cut=fields.index('-')
            found[unescape_mount(fields[4])]=(fields[0],fields[2],unescape_mount(fields[cut+2]))
        except (ValueError,IndexError):continue
    return found


@dataclass(frozen=True)
class UsbVolume:
    mount: str
    source: str
    uuid: str
    signature: tuple


def usb_volumes(tree,mounts):
    """USB transport is inherited by partitions; never treat an SD/system disk as USB."""
    result={}
    def visit(node,parent_usb=False):
        usb=parent_usb or node.get('tran')=='usb'
        target=node.get('mountpoint')
        if usb and isinstance(target,str) and target not in ('/','/boot','/boot/efi') and target in mounts:
            source=node.get('path') or node.get('name','')
            if source.startswith('/dev/') and mounts[target][2]==source:
                result[target]=UsbVolume(target,source,node.get('uuid') or '',mounts[target])
        for child in node.get('children') or []:visit(child,usb)
    for node in tree.get('blockdevices',[]):visit(node)
    return list(result.values())


class UsbNotReady(RuntimeError):
    """No mounted target yet; a pending start may wait for automount."""


def discover_usb(requested=''):
    try:
        run=subprocess.run(['lsblk','--json','--paths','--output','NAME,TYPE,TRAN,MOUNTPOINT,UUID'],
            capture_output=True,text=True,timeout=4,check=True)
        volumes=usb_volumes(json.loads(run.stdout),mount_table())
    except (OSError,ValueError,subprocess.SubprocessError) as exc:
        raise RuntimeError('无法检查U盘挂载状态：'+str(exc)) from exc
    if requested:
        target=str(Path(requested).resolve())
        volumes=[v for v in volumes if v.mount==target]
    if not volumes:raise UsbNotReady('未找到已挂载的U盘；插入并挂载U盘后再按9录像')
    if len(volumes)!=1:raise RuntimeError('检测到多个U盘分区，请在recording_config.json设置usb_mount')
    return volumes[0]


class UsbDirectory:
    """All file operations use a pinned directory on the verified USB filesystem."""
    def __init__(self,volume):
        self.volume=volume;self.fd=None
        self.check()
        flags=os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC
        root=os.open(volume.mount,flags)
        try:
            # Check the opened root BEFORE creating anything: an unmount racing
            # open() must not create robot-videos on the underlying system disk.
            dev=os.fstat(root).st_dev
            if f'{os.major(dev)}:{os.minor(dev)}'!=volume.signature[1]:
                raise RuntimeError('U盘挂载已改变，未创建录像目录')
            try:os.mkdir('robot-videos',mode=0o755,dir_fd=root)
            except FileExistsError:pass
            self.fd=os.open('robot-videos',flags,dir_fd=root)
            self.check()
        except BaseException:
            self.close();raise
        finally:os.close(root)

    def check(self):
        if (mount_table().get(self.volume.mount)!=self.volume.signature
                or not Path(self.volume.source).exists()):
            raise RuntimeError('U盘已移除或挂载改变，录像已停止')
        if self.fd is not None:
            dev=os.fstat(self.fd).st_dev
            if f'{os.major(dev)}:{os.minor(dev)}'!=self.volume.signature[1]:
                raise RuntimeError('U盘目录设备不匹配')

    def free_bytes(self):
        self.check();v=os.fstatvfs(self.fd)
        return v.f_bavail*v.f_frsize

    def reserve(self,name):
        self.check()
        fd=os.open(name,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o644,dir_fd=self.fd)
        os.close(fd)
        return f'/proc/self/fd/{self.fd}/{name}'

    def size(self,name):
        return os.stat(name,dir_fd=self.fd,follow_symlinks=False).st_size

    def finalize(self,name,metadata):
        self.check()
        fd=os.open(name,os.O_RDWR|os.O_NOFOLLOW,dir_fd=self.fd)
        try:os.fsync(fd)
        finally:os.close(fd)
        data=(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n').encode('utf8')
        fd=os.open(name+'.json',os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o644,dir_fd=self.fd)
        with os.fdopen(fd,'wb') as f:f.write(data);f.flush();os.fsync(f.fileno())

    def close(self):
        if self.fd is not None:os.close(self.fd);self.fd=None


class MjpegWriter:
    def __init__(self,path,fps,width,height):
        import cv2
        self.cv2=cv2;self.width=width;self.height=height;self.path=path
        self.writer=cv2.VideoWriter(path,cv2.VideoWriter_fourcc(*'MJPG'),fps,(width,height))
        if not self.writer.isOpened():
            self.writer.release();raise RuntimeError('无法创建MJPEG AVI录像，请检查U盘写权限及OpenCV编码支持')
        self.frames_submitted=0

    def write_rgb(self,rgb):
        import numpy as np
        image=np.frombuffer(rgb,dtype=np.uint8).reshape(self.height,self.width,3)
        self.writer.write(self.cv2.cvtColor(image,self.cv2.COLOR_RGB2BGR))
        self.frames_submitted+=1

    def close(self):
        self.writer.release()
        # VideoWriter.write has no success return value. Check the container's
        # frame count and both ends before claiming success. Seeking to the last
        # independently coded MJPEG frame avoids decoding minutes of video here.
        reader=self.cv2.VideoCapture(self.path)
        try:
            if (not reader.isOpened() or self.frames_submitted<1
                    or reader.get(self.cv2.CAP_PROP_FRAME_COUNT)!=self.frames_submitted):
                raise RuntimeError('录像文件帧数不完整，请检查U盘或编码器；未确认保存成功')
            ok,image=reader.read()
            if not ok or image is None or image.shape[:2]!=(self.height,self.width):
                raise RuntimeError('录像文件无法回放，请检查U盘或编码器；未确认保存成功')
            if self.frames_submitted>1:
                if not reader.set(self.cv2.CAP_PROP_POS_FRAMES,self.frames_submitted-1):
                    raise RuntimeError('录像文件尾帧无法定位；未确认保存成功')
                ok,image=reader.read()
                if (not ok or image is None or image.shape[:2]!=(self.height,self.width)
                        or reader.get(self.cv2.CAP_PROP_POS_FRAMES)!=self.frames_submitted):
                    raise RuntimeError('录像文件尾帧无法回放，请检查U盘或编码器；未确认保存成功')
        finally:reader.release()


class RecordingSession:
    def __init__(self,config,width,height,*,discover=discover_usb,lease_factory=UsbDirectory,writer_factory=MjpegWriter):
        self.config=config;self.width=width;self.height=height
        self.discover=discover;self.lease_factory=lease_factory;self.writer_factory=writer_factory
        self.lease=None;self.writer=None;self.name=None;self.segments=0;self.total_frames=0
        self.session=datetime.now(timezone.utc).strftime('OV5647_%Y%m%dT%H%M%SZ_')+uuid.uuid4().hex[:8]
        self.last_path='';self.next_frame=0.;self.checked_at=float('-inf')
        self.overlay_frames=0

    def start(self):
        volume=self.discover(self.config.usb_mount)
        self.lease=self.lease_factory(volume)
        try:self.check_space()
        except BaseException:self.lease.close();self.lease=None;raise

    def check_space(self):
        if self.lease.free_bytes()<self.config.minimum_free_mb*1024**2:
            raise RuntimeError(f'U盘剩余空间不足{self.config.minimum_free_mb}MB，录像已停止')

    def finish_segment(self,reason):
        if self.writer is None:return
        try:
            self.writer.close();self.writer=None
            self.lease.finalize(self.name,dict(camera='OV5647',width=self.width,height=self.height,
                codec='MJPG',container='AVI',fps=self.config.fps,frames_submitted=self.frames,
                overlay_frames=self.overlay_frames,data_overlay=bool(self.overlay_frames),
                first_capture_monotonic=self.first_capture,last_capture_monotonic=self.last_capture,
                started_monotonic=self.segment_start,ended_monotonic=self.last_written,
                reason=reason,usb_source=self.lease.volume.source,usb_uuid=self.lease.volume.uuid))
        finally:self.writer=None

    def write(self,rgb,captured,now,overlay=None):
        if rgb is None or not math.isfinite(captured) or not 0<=now-captured<1:
            raise RuntimeError('OV5647画面已断开或过期，录像已停止')
        if len(rgb)!=self.width*self.height*3:raise RuntimeError('OV5647录像帧长度错误')
        if now<self.next_frame:return
        if self.total_frames and now-self.next_frame>1:
            raise RuntimeError('录像编码跟不上实时画面，已停止；降低recording_config.json的fps后重试')
        self.lease.check()
        if now-self.checked_at>=.5:
            self.check_space();self.checked_at=now
        if self.writer is not None and (now-self.segment_start>=self.config.segment_seconds or
                self.lease.size(self.name)>=self.config.max_segment_mb*1024**2):
            self.finish_segment('segment limit')
        if self.writer is None:
            self.check_space();self.segments+=1
            self.name=f'{self.session}_{self.segments:04d}.avi'
            path=self.lease.reserve(self.name)
            self.writer=self.writer_factory(path,self.config.fps,self.width,self.height)
            self.last_path=str(Path(self.lease.volume.mount)/'robot-videos'/self.name)
            self.segment_start=now;self.frames=0;self.overlay_frames=0;self.first_capture=captured
            self.last_capture=captured;self.last_written=now
        self.writer.write_rgb(composite_rgb(rgb,self.width,self.height,overlay))
        self.frames+=1;self.total_frames+=1
        if overlay is not None:self.overlay_frames+=1
        self.last_capture=captured;self.last_written=now
        self.next_frame=(now if self.total_frames==1 else self.next_frame)+1/self.config.fps

    def stop(self,reason='operator stop'):
        try:self.finish_segment(reason)
        finally:
            if self.lease is not None:self.lease.close();self.lease=None


class RemoteRecordTrigger:
    """A new key9 count toggles once; baseline/reconnect/repeat never replay."""
    def __init__(self):self.previous=None
    def observe(self,nav,live):
        value=getattr(nav,'record_count',None)
        if not live or type(value) is not int or not 0<=value<=65535:
            self.previous=None;return False
        previous,self.previous=self.previous,value
        return previous is not None and value==previous+1


def recording_presentation(status):
    """One state-driven wording for the screen, saved overlay and browser."""
    status=status or {}
    state=status.get('state','idle')
    if status.get('error') or state=='error':
        return dict(text='录像失败：'+str(status.get('error') or '请重试'),
                    button='重试录像（9）',style='error',busy=False)
    if state=='starting':
        return dict(text='正在准备录像，尚未开始',button='准备中（9可取消）',style='preparing',busy=False)
    if state=='stopping':
        return dict(text='正在保存录像，请勿拔U盘',button='保存中…',style='saving',busy=True)
    if state=='recording' and status.get('active'):
        value=status.get('duration_s',0)
        seconds=int(value) if isinstance(value,(int,float)) and math.isfinite(value) and value>=0 else 0
        hours,rest=divmod(seconds,3600);minutes,seconds=divmod(rest,60)
        elapsed=f'{hours:02d}:{minutes:02d}:{seconds:02d}' if hours else f'{minutes:02d}:{seconds:02d}'
        return dict(text=f'● 正在录像  {elapsed}｜按9停止并保存',
                    button='停止并保存（9）',style='recording',busy=False)
    if state=='idle' and status.get('saved'):
        return dict(text='录像已停止，保存成功｜按9录制新视频',
                    button='重新录像（9）',style='saved',busy=False)
    return dict(text='未录像｜按9开始录像',button='开始录像（9）',style='idle',busy=False)


class VideoRecorder:
    """One latest-frame slot and a separate encoder worker; never blocks control."""
    START_WAIT_SECONDS = 5.0
    def __init__(self,config,width,height,*,session_factory=RecordingSession):
        self.config=config;self.width=width;self.height=height;self.session_factory=session_factory
        self.lock=threading.Lock();self.event=threading.Event();self.closed=threading.Event()
        self.latest=(None,0.,None,None,0.);self.requested=False;self.generation=0
        self.status=dict(state='idle',active=False,path='',frames=0,segments=0,error='',saved=False,duration_s=0.)
        self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()

    def offer(self,rgb,captured,overlay=None,*,fallback_rgb=None,fallback_captured=0.):
        """Keep annotated and raw capture times together; neither is refreshed."""
        with self.lock:self.latest=(rgb,captured,overlay,fallback_rgb,fallback_captured)

    @staticmethod
    def _select_frame(latest,now):
        rgb,captured,overlay,fallback_rgb,fallback_captured=latest
        # Recognition can expire while raw capture remains healthy. Select at
        # consumption time, including after USB startup, without making an old
        # image appear fresh or changing the strict camera disconnect limit.
        if ((rgb is None or not 0<=now-captured<1) and fallback_rgb is not None
                and 0<=now-fallback_captured<1):
            rgb,captured=fallback_rgb,fallback_captured
        return rgb,captured,overlay

    def fail(self,error):
        """Stop recording on a presentation failure; never send control commands."""
        with self.lock:
            self.requested=False
            self.status.update(state='error',active=False,saved=False,error=str(error))
        self.event.set()

    def toggle(self):
        with self.lock:
            if self.closed.is_set() or self.status['state']=='stopping':return False
            self.requested=not self.requested
            if self.requested:
                self.generation+=1
                self.status.update(state='starting',active=False,path='',frames=0,segments=0,
                                   duration_s=0.,saved=False,error='')
            else:self.status.update(state='stopping',active=False,saved=False,error='')
        self.event.set();return True

    def snapshot(self):
        with self.lock:return dict(self.status)

    def _run(self):
        session=None
        session_generation=None
        waiting_generation=None
        start_deadline=0.
        mount_retry_at=0.
        try:
            while not self.closed.is_set():
                with self.lock:
                    requested=self.requested;generation=self.generation
                    latest=self.latest
                try:
                    if session is not None and (not requested or generation!=session_generation):
                        session.stop()
                        saved=bool(session.total_frames and session.last_path)
                        with self.lock:
                            if (self.generation==session_generation and not self.requested
                                    and self.status['state']!='error'):
                                self.status.update(state='idle',active=False,saved=saved,
                                    path=session.last_path,frames=session.total_frames,
                                    segments=session.segments,duration_s=session.total_frames/self.config.fps)
                        session=None;session_generation=None
                        continue
                    if requested:
                        now=time.monotonic()
                        rgb,captured,overlay=self._select_frame(latest,now)
                        if session is None:
                            if waiting_generation!=generation:
                                waiting_generation=generation
                                start_deadline=now+self.START_WAIT_SECONDS
                                mount_retry_at=0.
                            if mount_retry_at and now>=start_deadline:
                                raise RuntimeError('等待U盘挂载超时，未启动录像')
                            if rgb is None or not 0<=now-captured<1:
                                if now>=start_deadline:
                                    raise RuntimeError('等待OV5647实时画面超时，未启动录像')
                                # Keep this one start request pending while capture
                                # recovers. Recheck cancellation and generation on
                                # every pass; never write a stale frame as live.
                                self.event.wait(.025);self.event.clear()
                                continue
                            if now<mount_retry_at:
                                self.event.wait(min(.025,mount_retry_at-now));self.event.clear()
                                continue
                            session=self.session_factory(self.config,self.width,self.height)
                            session_generation=generation
                            try:session.start()
                            except UsbNotReady:
                                # Discovery failed before any USB lease/file was
                                # opened. Only this transient start condition is
                                # retried; other errors and active failures stop.
                                session=None;session_generation=None
                                with self.lock:
                                    still_requested=self.requested and self.generation==generation
                                if not still_requested or self.closed.is_set():continue
                                if time.monotonic()>=start_deadline:
                                    raise RuntimeError('等待U盘挂载超时，未启动录像')
                                mount_retry_at=time.monotonic()+.1
                                continue
                        with self.lock:
                            still_requested=self.requested and self.generation==session_generation
                            latest=self.latest
                        if not still_requested or self.closed.is_set():continue
                        now=time.monotonic()
                        rgb,captured,overlay=self._select_frame(latest,now)
                        if not session.total_frames and now>=start_deadline:
                            raise RuntimeError('准备录像超时，未启动录像')
                        session.write(rgb,captured,now,overlay)
                        with self.lock:
                            if self.requested and self.generation==session_generation and self.status['state']!='error':
                                self.status.update(state='recording',active=True,path=session.last_path,
                                    frames=session.total_frames,segments=session.segments,
                                    duration_s=session.total_frames/self.config.fps,saved=False,error='')
                    elif not requested:
                        waiting_generation=None
                        with self.lock:
                            if self.generation==generation and not self.requested and self.status['state']!='error':
                                self.status.update(state='idle',active=False)
                except Exception as exc:
                    failed_generation=session_generation if session is not None else generation
                    if session is not None:
                        try:session.stop('recording error')
                        except Exception:pass
                        session=None;session_generation=None
                    with self.lock:
                        if self.generation==failed_generation:
                            self.requested=False;self.status.update(state='error',active=False,saved=False,error=str(exc))
                self.event.wait(.025);self.event.clear()
        finally:
            if session is not None:
                try:session.stop('viewer shutdown')
                except Exception as exc:
                    with self.lock:self.status.update(state='error',saved=False,error=str(exc))
            with self.lock:
                self.requested=False
                self.status['active']=False
                if self.status['state']!='error':self.status['state']='idle'

    def close(self):
        self.closed.set();self.event.set();self.thread.join(4)
        if self.thread.is_alive():raise TimeoutError('U盘录像写入未结束，请勿立即拔盘')
