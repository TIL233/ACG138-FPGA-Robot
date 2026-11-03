"""USB selection, pinned-target races, recording lifecycle and actual AVI decode."""
from pathlib import Path
from types import SimpleNamespace
from unittest import mock
import json,sys,tempfile,threading,time,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rk'))
import video_recording as v
from recording_overlay import OverlayPatch


class TestLease:
    """Filesystem test double only; production always uses UsbDirectory."""
    def __init__(self,root):
        self.root=root/'robot-videos';self.root.mkdir(exist_ok=True)
        self.volume=SimpleNamespace(mount=str(root),source='/dev/test-usb',uuid='test')
        self.free=1024**3;self.connected=True;self.closed=False;self.synthetic_size=None
    def check(self):
        if not self.connected:raise RuntimeError('U盘已移除')
    def free_bytes(self):self.check();return self.free
    def reserve(self,name):
        self.check();p=self.root/name
        with p.open('xb'):pass
        return str(p)
    def size(self,name):return self.synthetic_size or (self.root/name).stat().st_size
    def finalize(self,name,metadata):(self.root/(name+'.json')).write_text(json.dumps(metadata),encoding='utf8')
    def close(self):self.closed=True


class FakeWriter:
    def __init__(self,path,*_):self.path=Path(path);self.closed=False;self.frames=[]
    def write_rgb(self,rgb):
        self.frames.append(rgb)
        with self.path.open('ab') as f:f.write(b'frame'+rgb)
    def close(self):self.closed=True


class StorageTests(unittest.TestCase):
    def test_usb_transport_inherits_to_partition_and_rejects_system_sd_and_unmounted(self):
        tree={'blockdevices':[
            {'name':'/dev/mmcblk0','tran':None,'mountpoint':'/','children':[]},
            {'name':'/dev/mmcblk1','tran':None,'mountpoint':'/media/sd'},
            {'name':'/dev/sda','tran':'usb','children':[
                {'name':'/dev/sda1','mountpoint':'/media/U盘','uuid':'usb1'},
                {'name':'/dev/sda2','mountpoint':None}]},
            {'name':'/dev/sdb','tran':'usb','mountpoint':'/','uuid':'system'}]}
        mounts={'/':('1','179:0','/dev/mmcblk0'),'/media/U盘':('22','8:1','/dev/sda1'),
                '/media/sd':('3','179:2','/dev/mmcblk1')}
        volumes=v.usb_volumes(tree,mounts)
        self.assertEqual(len(volumes),1);self.assertEqual(volumes[0].uuid,'usb1')
        self.assertEqual(v.usb_volumes(tree,{}),[])

    def test_mountinfo_escapes_and_source_are_parsed_without_shell(self):
        mounts=v.mount_table('22 1 8:1 / /media/ubuntu/USB\\040DISK rw - vfat /dev/sda1 rw\ninvalid')
        self.assertEqual(mounts['/media/ubuntu/USB DISK'],('22','8:1','/dev/sda1'))

    def test_missing_and_ambiguous_usb_never_use_the_system_disk(self):
        tree={'blockdevices':[{'name':'/dev/sda','tran':'usb','mountpoint':'/media/a'},
                              {'name':'/dev/sdb','tran':'usb','mountpoint':'/media/b'}]}
        mounts={'/media/a':('1','8:0','/dev/sda'),'/media/b':('2','8:16','/dev/sdb')}
        with mock.patch.object(v.subprocess,'run',return_value=SimpleNamespace(stdout=json.dumps(tree))),mock.patch.object(v,'mount_table',return_value=mounts):
            with self.assertRaisesRegex(RuntimeError,'多个U盘'):v.discover_usb()
        with mock.patch.object(v.subprocess,'run',return_value=SimpleNamespace(stdout='{"blockdevices":[]}')),mock.patch.object(v,'mount_table',return_value={}):
            with self.assertRaisesRegex(RuntimeError,'未找到'):v.discover_usb()

    def test_unmount_racing_directory_open_cannot_create_on_system_disk(self):
        fake=SimpleNamespace(O_RDONLY=0,O_DIRECTORY=1,O_NOFOLLOW=2,O_CLOEXEC=4,
            open=mock.Mock(return_value=10),close=mock.Mock(),mkdir=mock.Mock(),
            fstat=mock.Mock(return_value=SimpleNamespace(st_dev=123)),major=lambda _:8,minor=lambda _:99)
        volume=v.UsbVolume('/media/usb','/dev/sda1','x',('5','8:1','/dev/sda1'))
        with mock.patch.object(v,'os',fake),mock.patch.object(v.UsbDirectory,'check'):
            with self.assertRaisesRegex(RuntimeError,'未创建'):v.UsbDirectory(volume)
        fake.mkdir.assert_not_called();fake.close.assert_called_once_with(10)

    def test_record_config_rejects_invalid_bounds_and_relative_target(self):
        for fields in ({'fps':True},{'fps':0},{'segment_seconds':0},{'minimum_free_mb':0},
                       {'max_segment_mb':2048},{'usb_mount':'videos'}):
            with self.subTest(fields=fields),self.assertRaises(ValueError):v.RecordingConfig(**fields)


class RecordingTests(unittest.TestCase):
    def session(self,root,**config):
        lease=TestLease(root)
        session=v.RecordingSession(v.RecordingConfig(**config),2,1,discover=lambda _:lease.volume,
            lease_factory=lambda _:lease,writer_factory=FakeWriter)
        session.start();return session,lease

    def test_time_segments_finalize_metadata_and_unique_names(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);s,lease=self.session(root,segment_seconds=2,fps=1)
            s.write(b'abcdef',10,10);s.write(b'abcdef',11,11)
            s.write(b'abcdef',12.1,12.1);s.stop()
            files=list(lease.root.glob('*.avi'));self.assertEqual(len(files),2)
            data=[json.loads(p.read_text()) for p in lease.root.glob('*.json')]
            self.assertEqual(sorted(x['frames_submitted'] for x in data),[1,2]);self.assertTrue(lease.closed)
            self.assertTrue(all(x['camera']=='OV5647' and x['codec']=='MJPG' for x in data))
            other,lease2=self.session(root);other.write(b'abcdef',20,20);other.stop()
            self.assertEqual(len(list(lease.root.glob('*.avi'))),3)

    def test_size_limit_splits_without_overwriting_and_frame_rate_is_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            s,lease=self.session(Path(directory),max_segment_mb=1)
            s.write(b'abcdef',10,10);s.write(b'abcdef',10.01,10.01)
            self.assertEqual(s.total_frames,1)
            lease.synthetic_size=1024**2
            s.write(b'abcdef',10.2,10.2);s.stop()
            self.assertEqual(s.segments,2);self.assertEqual(s.total_frames,2)

    def test_overlay_composites_after_rate_gate_preserves_input_and_is_counted_in_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            s,lease=self.session(Path(directory))
            rgb=bytes((40,80,120,0,0,0))
            red=OverlayPatch(1,0,1,1,bytes((255,0,0)),bytes((128,)))
            green=OverlayPatch(0,0,1,1,bytes((0,255,0)),bytes((255,)))
            with mock.patch.object(v,'composite_rgb',wraps=v.composite_rgb) as composite:
                s.write(rgb,9.9,10,red)
                writer=s.writer
                self.assertEqual(writer.frames,[bytes((40,80,120,128,0,0))])
                self.assertEqual(rgb,bytes((40,80,120,0,0,0)))
                s.write(rgb,9.91,10.01,green)
                self.assertEqual(composite.call_count,1)
                self.assertEqual(len(writer.frames),1)
                s.write(rgb,10.1,10.2)
            self.assertEqual(writer.frames,[bytes((40,80,120,128,0,0)),rgb])
            s.stop()
            metadata=json.loads(next(lease.root.glob('*.json')).read_text())
            self.assertEqual(metadata['overlay_frames'],1)
            self.assertEqual(metadata['frames_submitted'],2)
            self.assertEqual(metadata['first_capture_monotonic'],9.9)
            self.assertEqual(metadata['last_capture_monotonic'],10.1)

    def test_low_space_removed_volume_stale_and_incomplete_frames_fail_closed(self):
        for reason in ('space','removed','stale','incomplete'):
            with self.subTest(reason=reason),tempfile.TemporaryDirectory() as directory:
                s,lease=self.session(Path(directory));s.write(b'abcdef',10,10)
                if reason=='space':lease.free=0
                if reason=='removed':lease.connected=False
                with self.assertRaises(RuntimeError):
                    s.write(b'abc' if reason=='incomplete' else b'abcdef',9 if reason=='stale' else 11,11)
                s.stop('failure');self.assertTrue(lease.closed)

    def test_start_with_no_space_closes_usb_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            lease=TestLease(Path(directory));lease.free=0
            s=v.RecordingSession(v.RecordingConfig(),2,1,discover=lambda _:lease.volume,lease_factory=lambda _:lease)
            with self.assertRaisesRegex(RuntimeError,'剩余空间'):s.start()
            self.assertTrue(lease.closed);self.assertIsNone(s.lease)

    def test_sustained_encoder_overload_stops_instead_of_silently_speeding_video(self):
        with tempfile.TemporaryDirectory() as directory:
            s,lease=self.session(Path(directory));s.write(b'abcdef',10,10)
            with self.assertRaisesRegex(RuntimeError,'降低'):
                s.write(b'abcdef',11.5,11.5)
            s.stop('overload');self.assertTrue(lease.closed)

    def test_actual_mjpeg_avi_round_trip_preserves_geometry_frames_and_color(self):
        import cv2
        import numpy as np
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'roundtrip.avi'
            writer=v.MjpegWriter(str(target),10,32,24)
            rgb=np.zeros((24,32,3),np.uint8);rgb[:,:,0]=240
            for _ in range(4):writer.write_rgb(rgb.tobytes())
            writer.close();reader=cv2.VideoCapture(str(target))
            self.assertTrue(reader.isOpened());self.assertAlmostEqual(reader.get(cv2.CAP_PROP_FPS),10,places=1)
            frames=[]
            while True:
                ok,bgr=reader.read()
                if not ok:break
                frames.append(bgr)
            reader.release();self.assertEqual(len(frames),4)
            self.assertEqual(frames[0].shape,(24,32,3));self.assertGreater(frames[0][0,0,2],220)
            self.assertLess(frames[0][0,0,0],20)

    def test_saved_mjpeg_contains_bottom_left_overlay_and_preserves_other_pixels(self):
        import cv2
        import numpy as np
        with tempfile.TemporaryDirectory() as directory:
            lease=TestLease(Path(directory))
            s=v.RecordingSession(v.RecordingConfig(),64,48,discover=lambda _:lease.volume,
                lease_factory=lambda _:lease)
            s.start()
            source=np.zeros((48,64,3),np.uint8);source[:]=(220,20,20)
            patch=OverlayPatch(0,32,24,16,bytes((20,220,20))*24*16,bytes((255,))*24*16)
            s.write(source.tobytes(),9.9,10,patch);s.stop()
            reader=cv2.VideoCapture(str(next(lease.root.glob('*.avi'))))
            try:ok,bgr=reader.read()
            finally:reader.release()
            self.assertTrue(ok);self.assertEqual(bgr.shape,(48,64,3))
            self.assertGreater(int(bgr[40,12,1]),180);self.assertLess(int(bgr[40,12,2]),50)
            self.assertGreater(int(bgr[8,48,2]),180);self.assertLess(int(bgr[8,48,1]),50)
            metadata=json.loads(next(lease.root.glob('*.json')).read_text())
            self.assertEqual(metadata['overlay_frames'],1)

    def test_mjpeg_close_rejects_a_backend_that_silently_drops_later_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            writer=v.MjpegWriter(str(Path(directory)/'dropped.avi'),10,32,24)
            backend=writer.writer
            class DroppingBackend:
                submitted=0
                def write(self,image):
                    self.submitted+=1
                    if self.submitted==1:backend.write(image)
                def release(self):backend.release()
            writer.writer=DroppingBackend()
            for _ in range(30):writer.write_rgb(bytes((240,30,10))*32*24)
            with self.assertRaisesRegex(RuntimeError,'录像文件'):
                writer.close()

    def test_mjpeg_close_rejects_truncated_tail_with_a_readable_first_frame(self):
        import cv2
        import re
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'truncated.avi'
            writer=v.MjpegWriter(str(target),10,32,24)
            backend=writer.writer
            class TruncatingBackend:
                def write(self,image):backend.write(image)
                def release(self):
                    backend.release()
                    data=target.read_bytes()
                    chunks=list(re.finditer(rb'00dc.{4}\xff\xd8',data,re.DOTALL))
                    if len(chunks)!=4:raise AssertionError('expected four MJPEG chunks')
                    with target.open('r+b') as stream:stream.truncate(chunks[-1].start()+24)
            writer.writer=TruncatingBackend()
            for red in (40,80,120,160):writer.write_rgb(bytes((red,30,10))*32*24)
            with self.assertRaisesRegex(RuntimeError,'录像文件'):
                writer.close()
            reader=cv2.VideoCapture(str(target))
            try:
                self.assertEqual(reader.get(cv2.CAP_PROP_FRAME_COUNT),4)
                ok,first=reader.read()
                self.assertTrue(ok);self.assertEqual(first.shape,(24,32,3))
            finally:reader.release()

    def wait_for(self,predicate):
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            if predicate():return
            time.sleep(.01)
        self.fail('worker state timeout')

    def test_single_start_waits_for_a_fresh_camera_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            lease=TestLease(Path(directory))
            def factory(config,w,h):
                return v.RecordingSession(config,w,h,discover=lambda _:lease.volume,
                    lease_factory=lambda _:lease,writer_factory=FakeWriter)
            recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=factory)
            try:
                recorder.offer(b'abcdef',time.monotonic()-2)
                recorder.toggle();time.sleep(.08)
                self.assertEqual(recorder.snapshot()['state'],'starting')
                self.assertFalse(list(lease.root.glob('*.avi')))
                recorder.offer(b'abcdef',time.monotonic())
                self.wait_for(lambda:recorder.snapshot()['active'])
                self.assertTrue(list(lease.root.glob('*.avi')))
            finally:recorder.close()

    def mount_wait_recorder(self,root,mounted):
        lease=TestLease(root);attempts=[]
        def discover(_):
            attempts.append(time.monotonic())
            if mounted.is_set():return lease.volume
            # Exercise the real empty-discovery classification, not an error
            # message match that might also retry permission or encoder faults.
            with mock.patch.object(v.subprocess,'run',return_value=SimpleNamespace(stdout='{"blockdevices":[]}')):
                with mock.patch.object(v,'mount_table',return_value={}):return v.discover_usb()
        def factory(config,w,h):
            return v.RecordingSession(config,w,h,discover=discover,
                lease_factory=lambda _:lease,writer_factory=FakeWriter)
        return v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=factory),lease,attempts

    def test_single_start_waits_for_delayed_usb_mount_without_creating_a_video(self):
        with tempfile.TemporaryDirectory() as directory:
            mounted=threading.Event()
            recorder,lease,attempts=self.mount_wait_recorder(Path(directory),mounted)
            try:
                recorder.offer(b'abcdef',time.monotonic());recorder.toggle()
                self.wait_for(lambda:len(attempts)>=2)
                self.assertEqual(recorder.snapshot()['state'],'starting')
                self.assertFalse(list(lease.root.glob('*.avi')))
                self.assertGreaterEqual(attempts[1]-attempts[0],.08)
                mounted.set();recorder.offer(b'abcdef',time.monotonic())
                self.wait_for(lambda:recorder.snapshot()['active'])
                self.assertTrue(list(lease.root.glob('*.avi')))
            finally:recorder.close()

    def test_second_press_cancels_pending_usb_mount_start(self):
        with tempfile.TemporaryDirectory() as directory:
            mounted=threading.Event()
            recorder,lease,attempts=self.mount_wait_recorder(Path(directory),mounted)
            try:
                recorder.offer(b'abcdef',time.monotonic());recorder.toggle()
                self.wait_for(lambda:len(attempts)>=1)
                self.assertTrue(recorder.toggle())
                self.wait_for(lambda:recorder.snapshot()['state']=='idle')
                mounted.set();recorder.offer(b'abcdef',time.monotonic());time.sleep(.15)
                self.assertFalse(recorder.requested)
                self.assertFalse(list(lease.root.glob('*.avi')))
            finally:recorder.close()

    def test_usb_mount_timeout_does_not_replay_when_disk_later_appears(self):
        with tempfile.TemporaryDirectory() as directory:
            mounted=threading.Event()
            with mock.patch.object(v.VideoRecorder,'START_WAIT_SECONDS',.15):
                recorder,lease,attempts=self.mount_wait_recorder(Path(directory),mounted)
                try:
                    recorder.offer(b'abcdef',time.monotonic());recorder.toggle()
                    self.wait_for(lambda:recorder.snapshot()['state']=='error')
                    self.assertIn('等待U盘',recorder.snapshot()['error'])
                    self.assertGreaterEqual(len(attempts),2)
                    previous_attempts=len(attempts)
                    mounted.set();recorder.offer(b'abcdef',time.monotonic());time.sleep(.15)
                    self.assertFalse(recorder.requested)
                    self.assertEqual(len(attempts),previous_attempts)
                    self.assertFalse(list(lease.root.glob('*.avi')))
                finally:recorder.close()

    def test_fatal_recording_start_errors_are_not_retried(self):
        for error in (RuntimeError('ambiguous USB'),PermissionError('read-only USB'),
                      RuntimeError('no free space'),OSError('encoder unavailable')):
            with self.subTest(error=error):
                attempts=[]
                class FatalSession:
                    total_frames=0;last_path='';segments=0
                    def start(self):attempts.append(time.monotonic());raise error
                    def stop(self,*_):pass
                recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=lambda *_:FatalSession())
                try:
                    recorder.offer(b'abcdef',time.monotonic());recorder.toggle()
                    self.wait_for(lambda:recorder.snapshot()['state']=='error')
                    time.sleep(.12)
                    self.assertEqual(len(attempts),1)
                    self.assertFalse(recorder.requested)
                    self.assertIn(str(error),recorder.snapshot()['error'])
                finally:recorder.close()

    def test_slow_successful_start_expires_before_first_write_and_does_not_replay(self):
        for wait_for_mount in (False,True):
            with self.subTest(wait_for_mount=wait_for_mount),tempfile.TemporaryDirectory() as directory:
                entered=threading.Event();resume=threading.Event();attempts=[];sessions=[]
                class SlowReadyLease(TestLease):
                    def free_bytes(self):
                        entered.set();resume.wait(2)
                        return super().free_bytes()
                lease=SlowReadyLease(Path(directory))
                def discover(_):
                    attempts.append(time.monotonic())
                    if wait_for_mount and len(attempts)==1:
                        raise v.UsbNotReady('automount pending')
                    return lease.volume
                def factory(config,w,h):
                    session=v.RecordingSession(config,w,h,discover=discover,
                        lease_factory=lambda _:lease,writer_factory=FakeWriter)
                    sessions.append(session);return session
                with mock.patch.object(v.VideoRecorder,'START_WAIT_SECONDS',.15):
                    recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=factory)
                    try:
                        recorder.offer(b'abcdef',time.monotonic());recorder.toggle()
                        self.assertTrue(entered.wait(1),'start did not reach USB free-space check')
                        time.sleep(.18);recorder.offer(b'abcdef',time.monotonic());resume.set()
                        self.wait_for(lambda:lease.closed or bool(list(lease.root.glob('*.avi'))))
                        self.assertFalse(list(lease.root.glob('*.avi')))
                        self.wait_for(lambda:recorder.snapshot()['state']=='error')
                        self.assertIn('准备录像超时',recorder.snapshot()['error'])
                        self.assertTrue(lease.closed)
                        self.assertEqual(sessions[-1].total_frames,0)
                        self.assertFalse(list(lease.root.glob('*.avi')))
                        previous_attempts=len(attempts)
                        recorder.offer(b'abcdef',time.monotonic());time.sleep(.12)
                        self.assertFalse(recorder.requested)
                        self.assertEqual(len(attempts),previous_attempts)
                        self.assertFalse(list(lease.root.glob('*.avi')))
                    finally:resume.set();recorder.close()

    def test_cancelled_slow_start_remains_cancelled_when_it_returns_after_deadline(self):
        entered=threading.Event();resume=threading.Event();stopped=threading.Event();writes=[]
        class SlowStart:
            total_frames=0;last_path='';segments=0
            def start(self):entered.set();resume.wait(2)
            def write(self,*_):writes.append(True);self.total_frames+=1
            def stop(self,*_):stopped.set()
        with mock.patch.object(v.VideoRecorder,'START_WAIT_SECONDS',.05):
            recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=lambda *_:SlowStart())
            try:
                recorder.offer(b'abcdef',time.monotonic());recorder.toggle()
                self.assertTrue(entered.wait(1))
                time.sleep(.07);recorder.toggle();resume.set()
                self.wait_for(lambda:stopped.is_set() and recorder.snapshot()['state']=='idle')
                self.assertEqual(writes,[])
                self.assertEqual(recorder.snapshot()['error'],'')
                self.assertFalse(recorder.requested)
            finally:resume.set();recorder.close()

    def test_pending_start_times_out_without_replaying_on_later_frames(self):
        factory=mock.Mock()
        with mock.patch.object(v.VideoRecorder,'START_WAIT_SECONDS',.12):
            recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=factory)
            try:
                recorder.toggle()
                self.wait_for(lambda:recorder.snapshot()['state']=='error')
                self.assertIn('等待OV5647',recorder.snapshot()['error'])
                factory.assert_not_called()
                recorder.offer(b'abcdef',time.monotonic());time.sleep(.06)
                self.assertFalse(recorder.requested)
                factory.assert_not_called()
            finally:recorder.close()

    def test_second_press_cancels_pending_start_before_camera_recovers(self):
        factory=mock.Mock()
        recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=factory)
        try:
            recorder.toggle();time.sleep(.05)
            self.assertEqual(recorder.snapshot()['state'],'starting')
            recorder.toggle()
            self.wait_for(lambda:recorder.snapshot()['state']=='idle')
            recorder.offer(b'abcdef',time.monotonic());time.sleep(.06)
            factory.assert_not_called()
            self.assertFalse(recorder.requested)
        finally:recorder.close()

    def test_async_recording_stops_on_frame_loss_and_does_not_auto_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            lease=TestLease(Path(directory))
            def factory(config,w,h):
                return v.RecordingSession(config,w,h,discover=lambda _:lease.volume,lease_factory=lambda _:lease,writer_factory=FakeWriter)
            recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=factory)
            try:
                recorder.offer(b'abcdef',time.monotonic());recorder.toggle()
                self.wait_for(lambda:recorder.snapshot()['active'])
                recorder.offer(None,0.)
                self.wait_for(lambda:recorder.snapshot()['state']=='error')
                self.assertFalse(recorder.requested);self.assertTrue(lease.closed)
                recorder.offer(b'abcdef',time.monotonic());time.sleep(.06)
                self.assertFalse(recorder.snapshot()['active']);self.assertEqual(recorder.snapshot()['state'],'error')
            finally:recorder.close()

    def test_annotation_expiry_uses_fresh_raw_frame_and_preserves_capture_time_and_overlay(self):
        # If the worker keeps checking only the annotated capture time, a
        # healthy raw stream is stopped when inference crosses the age limit.
        clock=[10.]
        with tempfile.TemporaryDirectory() as directory:
            lease=TestLease(Path(directory));sessions=[]
            def factory(config,w,h):
                session=v.RecordingSession(config,w,h,discover=lambda _:lease.volume,
                    lease_factory=lambda _:lease,writer_factory=FakeWriter)
                sessions.append(session);return session
            patch=OverlayPatch(0,0,1,1,bytes((255,0,0)),bytes((255,)))
            with mock.patch.object(v,'time',SimpleNamespace(monotonic=lambda:clock[0])):
                recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=factory)
                try:
                    recorder.offer(b'abcdef',9.05,patch,fallback_rgb=b'ghijkl',fallback_captured=10.)
                    recorder.toggle();self.wait_for(lambda:recorder.snapshot()['active'])
                    writer=sessions[0].writer
                    self.assertEqual(writer.frames,[bytes((255,0,0))+b'def'])
                    self.assertEqual(sessions[0].last_capture,9.05)
                    clock[0]=10.2
                    self.wait_for(lambda:recorder.snapshot()['frames']>=2 or recorder.snapshot()['state']=='error')
                    self.assertEqual(recorder.snapshot()['state'],'recording',recorder.snapshot())
                    self.assertEqual(writer.frames[-1],bytes((255,0,0))+b'jkl')
                    self.assertEqual(sessions[0].last_capture,10.)
                    recorder.toggle();self.wait_for(lambda:recorder.snapshot()['saved'])
                    metadata=json.loads(next(lease.root.glob('*.json')).read_text())
                    self.assertEqual(metadata['first_capture_monotonic'],9.05)
                    self.assertEqual(metadata['last_capture_monotonic'],10.)
                    self.assertEqual(metadata['overlay_frames'],metadata['frames_submitted'])
                finally:recorder.close()

    def test_stale_annotation_can_start_with_fresh_raw_but_both_expired_stop(self):
        # A raw fallback must be selected before startup gating, then be
        # checked again at encode time; it must never hide a real disconnect.
        clock=[10.]
        with tempfile.TemporaryDirectory() as directory:
            lease=TestLease(Path(directory));sessions=[]
            def factory(config,w,h):
                session=v.RecordingSession(config,w,h,discover=lambda _:lease.volume,
                    lease_factory=lambda _:lease,writer_factory=FakeWriter)
                sessions.append(session);return session
            with mock.patch.object(v,'time',SimpleNamespace(monotonic=lambda:clock[0])):
                recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=factory)
                try:
                    recorder.offer(b'abcdef',8.,fallback_rgb=b'ghijkl',fallback_captured=9.9)
                    recorder.toggle();self.wait_for(lambda:recorder.snapshot()['active'])
                    writer=sessions[0].writer
                    self.assertEqual(writer.frames,[b'ghijkl'])
                    self.assertEqual(sessions[0].last_capture,9.9)
                    clock[0]=11.
                    self.wait_for(lambda:recorder.snapshot()['state']=='error')
                    self.assertIn('画面已断开或过期',recorder.snapshot()['error'])
                    self.assertEqual(writer.frames,[b'ghijkl'])
                    self.assertFalse(recorder.requested);self.assertTrue(lease.closed)
                    recorder.offer(b'mnopqr',11.,fallback_rgb=b'stuvwx',fallback_captured=11.)
                    time.sleep(.06)
                    self.assertEqual(recorder.snapshot()['state'],'error')
                    self.assertEqual(writer.frames,[b'ghijkl'])
                finally:recorder.close()

    def test_async_worker_keeps_overlay_capture_timestamp_and_frame_together(self):
        started=threading.Event();resume=threading.Event();writes=[]
        class PairSession:
            last_path='/test/paired.avi';total_frames=0;segments=1
            def start(self):started.set();resume.wait(1)
            def write(self,rgb,captured,now,overlay=None):
                writes.append((rgb,captured,overlay));self.total_frames+=1
            def stop(self,*_):pass
        session=PairSession()
        recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=lambda *_:session)
        first=OverlayPatch(0,0,1,1,bytes((255,0,0)),bytes((255,)))
        second=OverlayPatch(1,0,1,1,bytes((0,255,0)),bytes((255,)))
        a=(b'frameA',time.monotonic(),first)
        b=(b'frameB',time.monotonic(),second)
        try:
            recorder.offer(*a);recorder.toggle()
            self.assertTrue(started.wait(1),'worker did not start session')
            recorder.offer(*b);resume.set()
            self.wait_for(lambda:any(item==b for item in writes))
            self.assertTrue(writes)
            self.assertEqual(writes[0],b,'preparation must refresh the latest frame before encoding')
            self.assertTrue(all(item==a or item==b for item in writes))
        finally:resume.set();recorder.close()

    def test_async_render_failure_stops_session_and_does_not_auto_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            lease=TestLease(Path(directory))
            def factory(config,w,h):
                return v.RecordingSession(config,w,h,discover=lambda _:lease.volume,
                    lease_factory=lambda _:lease,writer_factory=FakeWriter)
            recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=factory)
            try:
                recorder.offer(b'abcdef',time.monotonic());recorder.toggle()
                self.wait_for(lambda:recorder.snapshot()['active'])
                recorder.fail('CJK font rendering failed')
                self.wait_for(lambda:lease.closed and recorder.snapshot()['state']=='error')
                self.assertFalse(recorder.requested)
                self.assertIn('CJK font rendering failed',recorder.snapshot()['error'])
                recorder.offer(b'abcdef',time.monotonic());time.sleep(.06)
                self.assertFalse(recorder.snapshot()['active'])
                self.assertEqual(recorder.snapshot()['state'],'error')
            finally:recorder.close()

    def test_render_failure_during_pending_encode_preserves_error_after_worker_stops(self):
        entered=threading.Event();resume=threading.Event();stopped=threading.Event()
        class BlockingSession:
            last_path='/test/pending.avi';total_frames=0;segments=1
            def start(self):pass
            def write(self,*_):
                entered.set();resume.wait(1);self.total_frames+=1
            def stop(self,*_):stopped.set()
        recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=lambda *_:BlockingSession())
        try:
            recorder.offer(b'abcdef',time.monotonic());recorder.toggle()
            self.assertTrue(entered.wait(1),'worker did not begin encoding')
            recorder.fail('overlay renderer failed during encode');resume.set()
            self.assertTrue(stopped.wait(1),'worker did not stop recording')
            self.wait_for(lambda:not recorder.snapshot()['active'])
            self.assertEqual(recorder.snapshot()['state'],'error')
            self.assertIn('overlay renderer failed during encode',recorder.snapshot()['error'])
        finally:resume.set();recorder.close()

    def test_saved_status_waits_for_finalization_and_stopping_clears_active_immediately(self):
        finalizing=threading.Event();resume=threading.Event()
        class SlowSaveSession:
            last_path='/test/saving.avi';total_frames=0;segments=1
            def start(self):pass
            def write(self,*_):self.total_frames+=1
            def stop(self,*_):finalizing.set();resume.wait(2)
        recorder=v.VideoRecorder(v.RecordingConfig(fps=5),2,1,session_factory=lambda *_:SlowSaveSession())
        try:
            initial=recorder.snapshot()
            self.assertFalse(initial['saved']);self.assertEqual(initial['duration_s'],0)
            recorder.offer(b'abcdef',time.monotonic());self.assertTrue(recorder.toggle())
            self.wait_for(lambda:recorder.snapshot()['active'])
            active=recorder.snapshot()
            self.assertAlmostEqual(active['duration_s'],active['frames']/5)
            self.assertFalse(active['saved'])
            self.assertTrue(recorder.toggle())
            stopping=recorder.snapshot()
            self.assertEqual(stopping['state'],'stopping')
            self.assertFalse(stopping['active']);self.assertFalse(stopping['saved'])
            self.assertTrue(finalizing.wait(1),'worker did not begin finalization')
            saving=recorder.snapshot()
            self.assertEqual(saving['state'],'stopping');self.assertFalse(saving['saved'])
            self.assertEqual(v.recording_presentation(saving)['style'],'saving')
            self.assertFalse(recorder.toggle(),'a new recording must wait for finalization')
            resume.set()
            self.wait_for(lambda:recorder.snapshot()['state']=='idle' and recorder.snapshot()['saved'])
            saved=recorder.snapshot()
            self.assertFalse(saved['active']);self.assertEqual(saved['path'],'/test/saving.avi')
            self.assertEqual(v.recording_presentation(saved)['style'],'saved')
        finally:resume.set();recorder.close()

    def test_failed_finalization_never_reports_saved_success(self):
        class FailedSaveSession:
            last_path='/test/failed.avi';total_frames=0;segments=1
            def start(self):pass
            def write(self,*_):self.total_frames+=1
            def stop(self,*_):raise RuntimeError('录像文件无法回放')
        recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=lambda *_:FailedSaveSession())
        try:
            recorder.offer(b'abcdef',time.monotonic());recorder.toggle()
            self.wait_for(lambda:recorder.snapshot()['active']);recorder.toggle()
            self.wait_for(lambda:recorder.snapshot()['state']=='error')
            failed=recorder.snapshot()
            self.assertFalse(failed['active']);self.assertFalse(failed['saved'])
            self.assertIn('无法回放',failed['error'])
            view=v.recording_presentation(failed)
            self.assertEqual(view['style'],'error');self.assertNotIn('保存成功',view['text'])
        finally:recorder.close()

    def test_new_start_clears_old_saved_path_frames_and_duration_before_preparation(self):
        preparing=threading.Event();resume=threading.Event();sessions=[]
        class ReusableSession:
            total_frames=0;segments=1
            def __init__(self,index):self.index=index;self.last_path=f'/test/take-{index}.avi'
            def start(self):
                if self.index==2:preparing.set();resume.wait(2)
            def write(self,*_):self.total_frames+=1
            def stop(self,*_):pass
        def factory(*_):
            session=ReusableSession(len(sessions)+1);sessions.append(session);return session
        recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=factory)
        try:
            recorder.offer(b'abcdef',time.monotonic());recorder.toggle()
            self.wait_for(lambda:recorder.snapshot()['active']);recorder.toggle()
            self.wait_for(lambda:recorder.snapshot()['state']=='idle' and recorder.snapshot()['saved'])
            previous=recorder.snapshot()
            self.assertEqual(previous['path'],'/test/take-1.avi')
            self.assertGreater(previous['frames'],0);self.assertGreater(previous['duration_s'],0)
            recorder.offer(b'abcdef',time.monotonic());self.assertTrue(recorder.toggle())
            self.assertTrue(preparing.wait(1),'worker did not begin second preparation')
            pending=recorder.snapshot()
            self.assertEqual(pending['state'],'starting');self.assertFalse(pending['active'])
            self.assertEqual(pending['path'],'');self.assertEqual(pending['frames'],0)
            self.assertEqual(pending['duration_s'],0);self.assertFalse(pending['saved'])
            self.assertEqual(v.recording_presentation(pending)['style'],'preparing')
            resume.set();self.wait_for(lambda:recorder.snapshot()['active'])
            self.assertEqual(recorder.snapshot()['path'],'/test/take-2.avi')
        finally:resume.set();recorder.close()

    def test_preparing_remains_visible_until_the_first_frame_has_been_written(self):
        encoding=threading.Event();resume=threading.Event()
        class SlowFirstFrameSession:
            last_path='/test/first-frame.avi';total_frames=0;segments=1
            def start(self):pass
            def write(self,*_):encoding.set();resume.wait(2);self.total_frames+=1
            def stop(self,*_):pass
        recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=lambda *_:SlowFirstFrameSession())
        try:
            recorder.offer(b'abcdef',time.monotonic());recorder.toggle()
            self.assertTrue(encoding.wait(1),'worker did not begin the first frame')
            pending=recorder.snapshot()
            self.assertEqual(pending['state'],'starting');self.assertFalse(pending['active'])
            self.assertFalse(pending['saved']);self.assertEqual(pending['frames'],0)
            self.assertEqual(v.recording_presentation(pending)['style'],'preparing')
            resume.set();self.wait_for(lambda:recorder.snapshot()['active'])
            self.assertGreater(recorder.snapshot()['frames'],0)
        finally:resume.set();recorder.close()

    def test_restart_during_old_failed_session_finalization_preserves_the_new_request(self):
        for finalize_error in (False,True):
            with self.subTest(finalize_error=finalize_error):
                finalizing=threading.Event();resume=threading.Event();sessions=[]
                class RestartSession:
                    total_frames=0;segments=1
                    def __init__(self,index):self.index=index;self.last_path=f'/test/restart-{index}.avi'
                    def start(self):pass
                    def write(self,*_):self.total_frames+=1
                    def stop(self,*_):
                        if self.index==1:
                            finalizing.set();resume.wait(2)
                            if finalize_error:raise RuntimeError('上一段保存失败')
                def factory(*_):
                    session=RestartSession(len(sessions)+1);sessions.append(session);return session
                recorder=v.VideoRecorder(v.RecordingConfig(),2,1,session_factory=factory)
                try:
                    recorder.offer(b'abcdef',time.monotonic());recorder.toggle()
                    self.wait_for(lambda:recorder.snapshot()['active'])
                    recorder.fail('旧录像呈现失败')
                    self.assertTrue(finalizing.wait(1),'old session did not begin finalizing')
                    recorder.offer(b'ghijkl',time.monotonic());self.assertTrue(recorder.toggle())
                    pending=recorder.snapshot()
                    self.assertEqual(pending['state'],'starting');self.assertTrue(recorder.requested)
                    self.assertEqual(pending['error'],'');self.assertFalse(pending['saved'])
                    resume.set()
                    self.wait_for(lambda:recorder.snapshot()['active'] and recorder.snapshot()['path']=='/test/restart-2.avi')
                    restarted=recorder.snapshot()
                    self.assertTrue(recorder.requested);self.assertEqual(restarted['state'],'recording')
                    self.assertEqual(restarted['error'],'');self.assertFalse(restarted['saved'])
                finally:resume.set();recorder.close()

    def test_remote9_baseline_repeat_gaps_and_reconnect_do_not_replay(self):
        trigger=v.RemoteRecordTrigger();nav=SimpleNamespace(record_count=2)
        self.assertFalse(trigger.observe(nav,True))
        nav.record_count=3;self.assertTrue(trigger.observe(nav,True))
        self.assertFalse(trigger.observe(nav,True))
        nav.record_count=6;self.assertFalse(trigger.observe(nav,True))
        self.assertFalse(trigger.observe(nav,False))
        nav.record_count=7;self.assertFalse(trigger.observe(nav,True))
        nav.record_count=8;self.assertTrue(trigger.observe(nav,True))
        nav.record_count=0;self.assertFalse(trigger.observe(nav,True))


if __name__=='__main__':unittest.main()
