from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rk'))
from protocol import CameraDiagnostic,CameraDiagnosticParser,crc16


def packet(seq=7,phase=6,index=12,chip=0x5640,nacks=3,flags=15):
    body=bytes((1,seq,phase,index,chip>>8,chip&255))+nacks.to_bytes(2,'little')+bytes((flags,11))
    return b'\x5a\xd4'+body+crc16(body).to_bytes(2,'little')+b'\r\n'


class CameraDiagnosticTests(unittest.TestCase):
    def test_v4_line_lengths_and_mixed_versions(self):
        import struct
        body=bytearray(packet()[2:-4]);body[0]=4
        body.extend(struct.pack('<10H',3000,800,2,0,0,2,32000,10,1279,1280))
        wire=b'\x5a\xd4'+body+crc16(body).to_bytes(2,'little')+b'\r\n'
        p=CameraDiagnosticParser()
        self.assertEqual(p.feed(wire[:32]),[])
        frames=p.feed(wire[32:]+packet())
        self.assertEqual((frames[0].line_bytes,frames[0].line_max_bytes),(1279,1280))
        self.assertIsNone(frames[1].line_bytes)

    def test_v3_duty_counters_fragmented_and_crc_recovery(self):
        import struct
        body=bytearray(packet()[2:-4]);body[0]=3
        body.extend(struct.pack('<8H',3000,800,2,0,0,2,32000,10))
        wire=b'\x5a\xd4'+body+crc16(body).to_bytes(2,'little')+b'\r\n'
        p=CameraDiagnosticParser()
        self.assertEqual(p.feed(wire[:29]),[])
        frame=p.feed(wire[29:])[0]
        self.assertEqual((frame.href_ticks,frame.vsync_ticks),(32000,10))
        bad=bytearray(wire);bad[29]^=1
        self.assertEqual(len(p.feed(bad+wire+packet())),2)
        self.assertGreater(p.bad_frames,0)

    def test_v2_counts_and_mixed_version_stream(self):
        body=bytearray(packet()[2:-4]);body[0]=2
        import struct
        body.extend(struct.pack('<6H',10000,1440,3,1400,1399,2))
        wire=b'\x5a\xd4'+body+crc16(body).to_bytes(2,'little')+b'\r\n'
        p=CameraDiagnosticParser()
        self.assertEqual(p.feed(wire[:20]),[])
        frames=p.feed(wire[20:]+packet())
        self.assertEqual((frames[0].pclk_ticks,frames[0].lines,frames[0].frames,
                          frames[0].packets_started,frames[0].packets_done,frames[0].dropped),
                         (10000,1440,3,1400,1399,2))
        self.assertIn('每100ms',frames[0].flow_summary)
        self.assertIsNone(frames[1].pclk_ticks)

    def test_split_frame_endianness_and_summary(self):
        p=CameraDiagnosticParser();data=packet(nacks=0x1234)
        self.assertEqual(p.feed(data[:8]),[])
        frame=p.feed(data[8:])[0]
        self.assertEqual((frame.seq,frame.index,frame.chip_id,frame.nacks,frame.last_nack_index),(7,12,0x5640,0x1234,11))
        self.assertIn('初始化12/253',frame.summary)

    def test_corrupt_frame_recovers_on_next_packet(self):
        bad=bytearray(packet());bad[7]^=1
        p=CameraDiagnosticParser()
        self.assertEqual(len(p.feed(bytes(bad)+packet())),1)
        self.assertGreater(p.bad_frames,0)

    def test_multiple_frames_noise_and_header_in_payload(self):
        p=CameraDiagnosticParser()
        frames=p.feed(b'noise'+packet(chip=0x5ad4)+packet(seq=8))
        self.assertEqual([f.seq for f in frames],[7,8])

    def test_no_ack_and_timeout_are_not_valid_chip_id(self):
        self.assertIn('无应答',CameraDiagnostic(0,6,0,0x5640,5,1,0).summary)
        self.assertIn('超时',CameraDiagnostic(0,5,0,0,0,129,255).summary)
        self.assertIn('不匹配',CameraDiagnostic(0,6,0,0x1234,0,7,255).summary)
        self.assertIn('初始化完成',CameraDiagnostic(0,7,0,0x5640,0,31,255).summary)

    def test_late_camera_recovery_does_not_hide_current_initialization_success(self):
        text=CameraDiagnostic(0,7,0,0xffff,12,17,0).summary
        self.assertIn('开机芯片ID读取无应答',text)
        self.assertIn('当前初始化完成',text)


if __name__=='__main__':unittest.main()
