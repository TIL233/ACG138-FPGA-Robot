"""Decode diagnostic bundles produced by the actual top-level RTL test.

Run run_rtl.ps1 first. This checks the producer's packed bus against the RK
consumer, in addition to the separate real-UART CRC/layout test.
"""
from pathlib import Path
import math
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'rk'))
from protocol import LidarPlanningTelemetryParser, crc16


class PlanningRtlSnapshotTests(unittest.TestCase):
    def test_rtl_bus_layout_observation_drive_hold_and_follow(self):
        path=Path(__file__).with_name('planning_diagnostics.hex')
        if not path.exists():
            self.skipTest('run ModelSim tb_lidar_advanced_top first')
        samples={}
        for line in path.read_text().splitlines():
            name,hex_value=line.split()
            payload=bytearray(int(hex_value,16).to_bytes(40,'little'))
            payload[0:2]=bytes((1,42))
            body=bytes(payload)
            frame=b'\x5a\xc8'+body+crc16(body).to_bytes(2,'little')+b'\r\n'
            parser=LidarPlanningTelemetryParser();decoded=parser.feed(frame)
            self.assertEqual(parser.bad_frames,0,name)
            self.assertEqual(len(decoded),1,name)
            samples[name]=decoded[0]
        self.assertEqual(set(samples),{'drive','observe','hold','follow'})
        for name in ('drive','observe'):
            s=samples[name]
            self.assertTrue(s.result_valid,name)
            self.assertEqual((s.candidate,s.candidate_count,s.target_l,s.target_r),(7,9,12,12))
            self.assertEqual((s.admissible_mask,s.unknown_mask),(0x1ff,0))
            speed=math.ceil(12*50000/7640)
            stop=200+math.ceil(speed*410/1000)+math.ceil(speed*speed/400)
            self.assertEqual((s.stop_mm,s.slow_mm,s.speed_mm_s),(stop,stop+200,speed))
            self.assertEqual((s.age_ms,s.cycles,s.goal_cd),(90,443,0))
        self.assertTrue(samples['drive'].actual_takeover)
        self.assertFalse(samples['observe'].actual_takeover)
        held=samples['hold']
        self.assertTrue(held.actual_takeover)
        self.assertTrue(held.flags&0x80)
        self.assertFalse(held.result_valid)
        self.assertEqual((held.candidate,held.target_l,held.target_r),(255,0,0))
        followed=samples['follow']
        self.assertTrue(followed.flags&0x1000)
        self.assertFalse(followed.actual_takeover)
        self.assertFalse(followed.result_valid)
        self.assertEqual((followed.candidate,followed.target_l,followed.target_r),(255,0,0))


if __name__=='__main__':unittest.main()
