"""A CRC-protected current preset survives frame loss without replaying keys."""
from pathlib import Path
import struct,sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rk'))
from protocol import NavTelemetryParser,TelemetryParser,crc16

def frame(version,speed=8):
    body=struct.pack('<BBBBBBiihBB',version,17,0x31,2,3,0xc0,-400,500,-600,8,0)
    if version>=2:body+=struct.pack('<H',0x1234)
    if version>=3:body+=struct.pack('<H',0x5678)
    if version>=4:body+=bytes([speed])
    if version>=5:body+=struct.pack('<H',0xabcd)
    return b'\x5a\xc3'+body+struct.pack('<H',crc16(body))+b'\r\n'

class FollowSpeedProtocolTests(unittest.TestCase):
    def test_v5_record_generation_is_crc_protected_and_keeps_follow_speed(self):
        packet=frame(5,10);self.assertEqual(len(packet),31)
        for split in range(len(packet)+1):
            p=NavTelemetryParser();out=p.feed(packet[:split])+p.feed(packet[split:])
            self.assertEqual(len(out),1)
            self.assertEqual((out[0].follow_speed,out[0].record_count,out[0].stop_epoch),(10,0xabcd,3))
            self.assertEqual((out[0].follow_count,out[0].nav_mode_count),(0x1234,0x5678))

    def test_old_versions_keep_optional_speed_unset(self):
        for version,length in [(1,24),(2,26),(3,28)]:
            packet=frame(version);self.assertEqual(len(packet),length)
            t=NavTelemetryParser().feed(packet)[0]
            self.assertIsNone(t.follow_speed);self.assertEqual(t.stop_epoch,3)

    def test_v4_all_splits_preserve_stop_start_events_coordinates_and_speed(self):
        packet=frame(4,12);self.assertEqual(len(packet),29)
        for split in range(len(packet)+1):
            p=NavTelemetryParser();out=p.feed(packet[:split])+p.feed(packet[split:])
            self.assertEqual(len(out),1);t=out[0]
            self.assertEqual((t.follow_speed,t.stop_epoch,t.follow_count,t.nav_mode_count),(12,3,0x1234,0x5678))
            self.assertEqual((t.state,t.x_mm,t.y_mm,t.heading),(1,-400,500,-600))
            self.assertEqual(p.bad_frames,0)

    def test_crc_recovery_and_mixed_versions_do_not_block_ordinary_frames(self):
        bad=bytearray(frame(4,12));bad[24]^=1
        ordinary_body=struct.pack('<BBBBBBHHhhhhhH',1,7,0,0x3f,0,0,1000,1000,0,0,0,0,0,0)
        ordinary=b'\x5a\xa5'+ordinary_body+struct.pack('<H',crc16(ordinary_body))+b'\r\n'
        stream=bytes(bad)+frame(3)+ordinary+frame(4,10)
        for split in range(len(stream)+1):
            p=NavTelemetryParser();out=p.feed(stream[:split])+p.feed(stream[split:])
            self.assertEqual([t.follow_speed for t in out],[None,10]);self.assertEqual(p.bad_frames,1)
            legacy=TelemetryParser();old=legacy.feed(stream[:split])+legacy.feed(stream[split:])
            self.assertEqual([t.seq for t in old],[7])

if __name__=='__main__':unittest.main()
