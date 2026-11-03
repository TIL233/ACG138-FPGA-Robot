import struct
import threading
import unittest
from unittest import mock
from types import SimpleNamespace
from robot_rebuild.rk import protocol as p


class UsbTransportTests(unittest.TestCase):
    def reader(self):
        r=p.PosixTelemetryLink('unused')
        r.fd=17
        r.lidar_telemetry=p.LidarTelemetry(1,0xc0,0,65535,65535,65535,65535,0,0,0)
        r.last_lidar_frame_rx=10.
        return r

    def test_encoding_preserves_raw_bytes_and_cannot_expose_command_headers(self):
        raw=p.command_packet(1,True,1,8)+b'\xa5\xb7\x7d\x7e'
        packet=p.lidar_forward_packet(7,raw)
        self.assertEqual(packet[:2],b'\xa5\xb7');self.assertEqual(packet[-1:],b'\x7e')
        self.assertNotIn(b'\xa5',packet[2:-1]);self.assertNotIn(b'\x7e',packet[2:-1])
        data=bytearray();esc=False
        for b in packet[2:-1]:
            if esc:data.append(b^0x20);esc=False
            elif b==0x7d:esc=True
            else:data.append(b)
        expected=bytes((1,7,0,len(raw)))+raw
        self.assertEqual(bytes(data),expected+struct.pack('<H',p.crc16(expected)))
        self.assertEqual(p.lidar_forward_packet(0,b'',reset=True)[:2],b'\xa5\xb7')
        for payload,reset in ((b'',False),(bytes(129),False),(b'x',True)):
            with self.assertRaises(ValueError):p.lidar_forward_packet(0,payload,reset=reset)

    def test_requires_fresh_new_firmware_even_before_any_scan(self):
        r=self.reader()
        self.assertTrue(r.lidar_bridge_ready(10.2))
        self.assertFalse(r.lidar_live(10.2))
        for now in (9.9,10.5):self.assertFalse(r.lidar_bridge_ready(now))
        r.lidar_telemetry=p.LidarTelemetry(1,0x30,0,65535,65535,65535,65535,0,0,0)
        self.assertFalse(r.lidar_bridge_ready(10.2))

    def test_bounded_raw_writes_and_partial_write_require_another_reset(self):
        r=self.reader()
        with mock.patch.object(r,'_pending_output_bytes',return_value=0),mock.patch('os.write',side_effect=lambda fd,b:len(b)) as write:
            self.assertFalse(r.send_lidar_bytes(b'raw',now=10.1))
            self.assertTrue(r.send_lidar_bytes(b'',reset=True,now=10.1))
            self.assertTrue(r.send_lidar_bytes(b'\xa5'*128,now=10.1))
            self.assertEqual(write.call_count,2)
            with mock.patch.object(r,'_pending_output_bytes',return_value=384):
                self.assertFalse(r.send_lidar_bytes(b'raw',now=10.1))
            write.side_effect=lambda fd,b:len(b)-1
            with self.assertRaises(OSError):r.send_lidar_bytes(b'raw',now=10.1)
            write.side_effect=lambda fd,b:len(b)
            self.assertFalse(r.send_lidar_bytes(b'raw',now=10.1))
            self.assertTrue(r.send_lidar_bytes(b'',reset=True,now=10.1))

    def test_raw_sender_yields_to_a_waiting_control_writer(self):
        r=self.reader()
        with mock.patch.object(r,'_pending_output_bytes',return_value=0),mock.patch('os.write',side_effect=lambda fd,b:len(b)) as write:
            self.assertTrue(r.send_lidar_bytes(b'',reset=True,now=10.1))
            r._control_pending.set()
            self.assertFalse(r.send_lidar_bytes(b'raw',now=10.1))
            r._control_pending.clear()
            r._write_serial_packet(p.command_packet(0,False,0,0))
            self.assertEqual(write.call_args.args[1],p.command_packet(0,False,0,0))

    def test_old_capability_frame_repetition_does_not_extend_readiness(self):
        r=self.reader();r.last_lidar_frame_seq=1
        body=struct.pack('<BB9H',1,1,0xc0,0,65535,65535,65535,65535,0,0,0)
        frame=b'\x5a\xb7'+body+struct.pack('<H',p.crc16(body))+b'\r\n'
        # Exercise real poll parsing without opening a device.
        with mock.patch.dict('sys.modules',{'fcntl':SimpleNamespace(),'termios':SimpleNamespace(),'tty':SimpleNamespace()}),mock.patch('os.read',return_value=frame):
            r.poll(10.6)
        self.assertFalse(r.lidar_bridge_ready(10.6))

    def test_concurrent_control_does_not_interleave_with_a_sensor_frame(self):
        r=self.reader();entered=threading.Event();release=threading.Event();writes=[];errors=[]
        def write(fd,data):
            if data.startswith(b'\xa5\xb7') and len(data)>20:
                entered.set();release.wait(1)
            writes.append(bytes(data));return len(data)
        def raw():
            try:r.send_lidar_bytes(b'x'*128,now=10.1)
            except Exception as exc:errors.append(exc)
        def stop():
            try:r._write_serial_packet(p.command_packet(1,False,0,0))
            except Exception as exc:errors.append(exc)
        with mock.patch.object(r,'_pending_output_bytes',return_value=0),mock.patch('os.write',side_effect=write):
            r.send_lidar_bytes(b'',reset=True,now=10.1);writes.clear()
            a=threading.Thread(target=raw);b=threading.Thread(target=stop)
            try:
                a.start();self.assertTrue(entered.wait(1));b.start()
                self.assertTrue(r._control_pending.wait(1))
                self.assertFalse(r.send_lidar_bytes(b'next',now=10.1))
            finally:
                release.set();a.join(1);b.join(1)
        self.assertFalse(a.is_alive() or b.is_alive());self.assertFalse(errors)
        self.assertEqual(writes,[p.lidar_forward_packet(1,b'x'*128),p.command_packet(1,False,0,0)])


if __name__=='__main__':unittest.main()
