"""C8 diagnostics stay read-only, and stale suggestions never look active."""
import importlib
from dataclasses import replace
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

from robot_rebuild.rk import protocol as p


def frame(**overrides):
    fields = dict(version=1, seq=7, flags=0x19, timestamp_ms=123456,
                  scan_id=42, generation=13, age_ms=275, stop_mm=620,
                  slow_mm=820, clearance_mm=80, speed_mm_s=79, cycles=301,
                  candidate=3, candidate_count=9, target_l=3, target_r=6,
                  admissible_mask=0x18, unknown_mask=0x100, goal_cd=-1200,
                  command_epoch=18, latency_budget_ms=595)
    fields.update(overrides)
    body = struct.pack('<BBHI8HBBhhHHhHH', *fields.values())
    return b'\x5a\xc8' + body + struct.pack('<H', p.crc16(body)) + b'\r\n'


class PlanningProtocolTests(unittest.TestCase):
    def parser(self):
        self.assertTrue(hasattr(p, 'LidarPlanningTelemetryParser'), 'missing C8 parser')
        return p.LidarPlanningTelemetryParser()

    def test_frozen_layout_all_fragment_boundaries_and_signed_goal(self):
        raw = frame()
        self.assertEqual(len(raw), 46)
        for split in range(47):
            parser = self.parser()
            samples = parser.feed(raw[:split]) + parser.feed(raw[split:])
            self.assertEqual(len(samples), 1)
            t = samples[0]
            self.assertEqual((t.timestamp_ms, t.scan_id, t.generation, t.age_ms), (123456,42,13,275))
            self.assertEqual((t.cycles,t.candidate,t.target_l,t.target_r,t.goal_cd), (301,3,3,6,-1200))
            self.assertEqual((t.command_epoch,t.latency_budget_ms), (18,595))

    def test_crc_version_terminator_noise_recover_to_next_frame(self):
        corrupt = bytearray(frame()); corrupt[10] ^= 1
        terminator = frame()[:-1] + b'!'
        parser = self.parser()
        result = parser.feed(b'\0'*100000 + bytes(corrupt) + frame(version=2) + terminator + frame(seq=9))
        self.assertEqual([t.seq for t in result], [9])
        self.assertEqual(parser.bad_frames, 3)
        self.assertLessEqual(len(parser.buffer), 1)
        self.assertEqual(parser.feed(b'noise\x5a'), [])
        self.assertEqual(len(parser.feed(frame()[1:])), 1)

    def test_semantic_rejection_and_valid_absent_result(self):
        cases = [dict(flags=0x4019), dict(candidate_count=8), dict(candidate=9),
                 dict(admissible_mask=512), dict(unknown_mask=512),
                 dict(flags=1), dict(flags=1,candidate=255,target_l=1,target_r=0),
                 dict(target_l=-3), dict(candidate=8), dict(unknown_mask=0x108)]
        for fields in cases:
            with self.subTest(fields=fields):
                parser = self.parser()
                self.assertEqual(parser.feed(frame(**fields)), [])
                self.assertEqual(parser.bad_frames, 1)
        samples = self.parser().feed(frame(flags=1,candidate=255,target_l=0,target_r=0))
        self.assertEqual(len(samples), 1)

    def test_both_reader_paths_expire_frozen_stamp_even_when_seq_advances(self):
        self.parser()
        port = mock.Mock(in_waiting=46)
        with mock.patch.dict(sys.modules, {'serial':SimpleNamespace(Serial=lambda *a,**k:port)}):
            serial = p.Link('unused')
        posix = p.PosixTelemetryLink('unused'); posix.fd=17
        modules = {'termios':SimpleNamespace(error=OSError), 'fcntl':SimpleNamespace(), 'tty':SimpleNamespace()}
        with mock.patch.dict(sys.modules, modules), mock.patch('os.read') as read, mock.patch('os.close'):
            for now,seq,stamp,live in ((10,1,100,True),(10.2,2,100,True),(10.5,3,100,False),(10.6,4,120,True)):
                port.read.return_value = read.return_value = frame(seq=seq,timestamp_ms=stamp)
                with mock.patch.object(p.time,'monotonic',return_value=now):serial.poll()
                posix.poll(now)
                for link in (serial,posix):
                    self.assertEqual(link.lidar_planning_live(now), live)
                    self.assertFalse(link.lidar_planning_live(9))
            read.return_value=b'';posix.poll(10.7)
            self.assertIsNone(posix.lidar_planning_telemetry)
            self.assertEqual(posix.lidar_planning_parser.buffer,b'')
            self.assertIsNone(posix.last_lidar_planning_stamp)
            serial.close()
            self.assertIsNone(serial.lidar_planning_telemetry)
            self.assertFalse(serial.lidar_planning_live(10.7))
        port.write.assert_not_called()

    def test_serial_read_error_clears_snapshot_and_partial_bytes(self):
        self.parser()
        port=mock.Mock(in_waiting=46)
        with mock.patch.dict(sys.modules, {'serial':SimpleNamespace(Serial=lambda *a,**k:port)}):
            link=p.Link('unused')
        port.read.return_value=frame()+frame()[:12]
        link.poll()
        port.read.side_effect=OSError('unplugged')
        with self.assertRaises(OSError):link.poll()
        self.assertIsNone(link.lidar_planning_telemetry)
        self.assertEqual(link.lidar_planning_parser.buffer,b'')

    def test_takeover_requires_all_gates_and_rejects_conflicting_results(self):
        active=0x203f
        for flags in (active & ~1, active & ~2, active & ~4, active & ~8,
                      active & ~16, active & ~0x2000, active|0x1000,
                      active|0x100, active|0x400, active|0x800, 0x119,0x419,0x819):
            with self.subTest(flags=flags):
                self.assertEqual(self.parser().feed(frame(flags=flags)),[])

    def test_protective_hold_takeover_accepts_no_result_but_requires_all_gates(self):
        hold=0x20a7  # capability, enabled, validated, actual, hold, original control
        absent=dict(candidate=255,target_l=0,target_r=0)
        for extra in (0,8,0x100,0x200,0x400,0x800,0xf00):
            with self.subTest(extra=extra):
                samples=self.parser().feed(frame(flags=hold|extra,**absent))
                self.assertEqual(len(samples),1)
                self.assertTrue(samples[0].actual_takeover)
                self.assertFalse(samples[0].result_valid)
        for flags in (hold & ~1,hold & ~2,hold & ~4,hold & ~0x2000,
                      hold|0x1000,hold & ~0x80):
            with self.subTest(flags=flags):
                self.assertEqual(self.parser().feed(frame(flags=flags,**absent)),[])
        for fields in (dict(candidate=3),dict(target_l=1),dict(target_r=1)):
            self.assertEqual(self.parser().feed(frame(flags=hold,**(absent|fields))),[])


class PlanningPresentationTests(unittest.TestCase):
    def module(self):
        self.assertTrue(hasattr(p,'LidarPlanningTelemetryParser'), 'missing C8 parser')
        return importlib.import_module('robot_rebuild.rk.lidar_planning')

    def test_observation_is_explicit_and_takeover_uses_actual_flag(self):
        module=self.module();t=p.LidarPlanningTelemetryParser().feed(frame())[0]
        line=module.planning_summary(t,True)
        self.assertIn('雷达规划：观察',line);self.assertIn('建议左弧',line)
        self.assertIn('安全62',line);self.assertNotIn('\n',line)
        active=p.LidarPlanningTelemetryParser().feed(frame(flags=0x203f))[0]
        self.assertIn('接管',module.planning_summary(active,True))
        self.assertNotIn('建议',module.planning_summary(active,True))
        details='\n'.join(module.planning_details(t,True))
        for expected in ('候选 3/9','301','275ms','未验收','0x018','0x100','595ms','18','13','-12.00'):
            self.assertIn(expected,details)

    def test_waiting_and_legacy_do_not_reuse_suggestions_or_targets(self):
        module=self.module();t=p.LidarPlanningTelemetryParser().feed(frame())[0]
        self.assertIsNone(module.planning_summary(None,False))
        self.assertEqual(module.planning_details(None,False),[])
        self.assertIsNone(module.planning_snapshot(None,False))
        text=module.planning_summary(t,False)
        self.assertIn('等待',text);self.assertNotIn('左弧',text);self.assertNotIn('62',text)
        snapshot=module.planning_snapshot(t,False)
        self.assertFalse(snapshot['live']);self.assertFalse(snapshot['actual_takeover'])
        self.assertIsNone(snapshot['candidate']);self.assertIsNone(snapshot['target_l'])

    def test_candidate_directions_no_path_unknown_fault_and_revoked_are_distinct(self):
        module=self.module()
        for candidate,direction in ((0,'左弧'),(1,'直行'),(2,'右弧')):
            t=p.LidarPlanningTelemetryParser().feed(frame(candidate=candidate,admissible_mask=1<<candidate,unknown_mask=0))[0]
            self.assertIn(direction,module.planning_summary(t,True))
        for flag,label in ((0x100,'无可行路'),(0x200,'未知'),(0x400,'撤销'),(0x800,'故障')):
            t=p.LidarPlanningTelemetryParser().feed(frame(flags=1|flag,candidate=255,target_l=0,target_r=0))[0]
            self.assertIn(label,'\n'.join(module.planning_details(t,True)))

    def test_protective_hold_reports_takeover_without_a_feasible_route(self):
        module=self.module()
        base=p.LidarPlanningTelemetryParser().feed(frame())[0]
        for extra,label in ((0,'保护停车'),(0x100,'无可行路'),(0x200,'等待地图'),
                            (0x400,'撤销'),(0x800,'故障')):
            t=replace(base,flags=0x20a7|extra,candidate=255,target_l=0,target_r=0)
            text=module.planning_summary(t,True)
            self.assertIn('雷达规划：接管｜保护停车',text)
            self.assertIn(label,text)
            self.assertNotIn('建议',text)
            self.assertNotIn('左弧',text)
            self.assertNotIn('直行',text)
            details='\n'.join(module.planning_details(t,True))
            self.assertIn('保护停车目标',details)
            self.assertIn('候选 --/9',details)
            snapshot=module.planning_snapshot(t,True)
            self.assertTrue(snapshot['actual_takeover'])
            self.assertFalse(snapshot['result_valid'])
            self.assertEqual((snapshot['candidate'],snapshot['target_l'],snapshot['target_r']),(255,0,0))

    def test_observation_hold_is_a_stop_suggestion_and_never_claims_execution(self):
        module=self.module()
        for extra,label in ((0,'建议停车'),(0x100,'无可行路'),(0x200,'等待地图'),
                            (0x400,'撤销'),(0x800,'故障')):
            samples=p.LidarPlanningTelemetryParser().feed(frame(flags=0x81|extra,candidate=255,target_l=0,target_r=0))
            self.assertEqual(len(samples),1)
            text=module.planning_summary(samples[0],True)
            self.assertIn('雷达规划：观察｜建议停车',text)
            self.assertIn(label,text)
            self.assertNotIn('保护停车',text)
            self.assertNotIn('接管',text)
            self.assertNotIn('等待有效结果',text)
            snapshot=module.planning_snapshot(samples[0],True)
            self.assertFalse(snapshot['actual_takeover'])
            self.assertFalse(snapshot['result_valid'])


if __name__=='__main__':unittest.main()
