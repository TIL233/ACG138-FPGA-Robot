"""Offline exporter contracts using synthetic wire data and real filesystem I/O."""
import csv
from dataclasses import fields
import hashlib
import importlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

from robot_rebuild.rk.protocol import LidarPlanningTelemetry, crc16


ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools' / 'export_lidar_planning.py'


def planning_frame(**overrides):
    """Synthetic C8 v1 fixture; no hardware or control link is involved."""
    values = dict(version=1, seq=7, flags=0x19, timestamp_ms=123456,
                  scan_id=42, generation=13, age_ms=275, stop_mm=620,
                  slow_mm=820, clearance_mm=80, speed_mm_s=79, cycles=301,
                  candidate=3, candidate_count=9, target_l=3, target_r=6,
                  admissible_mask=0x18, unknown_mask=0x100, goal_cd=-1200,
                  command_epoch=18, latency_budget_ms=595)
    values.update(overrides)
    body = struct.pack('<BBHI8HBBhhHHhHH', *values.values())
    return b'\x5a\xc8' + body + struct.pack('<H', crc16(body)) + b'\r\n'


def old_motion_frame():
    body = struct.pack('<BBHI12hii', 1, 2, 0, 100, *([0] * 14))
    return b'\x5a\xe5' + body + struct.pack('<H', crc16(body)) + b'\r\n'


class ExportPlanningTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(TOOL.is_file(), 'missing offline C8 exporter')
        self.export = importlib.import_module(
            'robot_rebuild.tools.export_lidar_planning').export_capture
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.source = self.base / 'synthetic-uart.bin'

    def run_cli(self, output, source=None):
        env = dict(os.environ, PYTHONIOENCODING='utf-8',
                   PYTHONDONTWRITEBYTECODE='1')
        return subprocess.run(
            [sys.executable, '-S', str(TOOL), '--from-binary',
             str(source or self.source), '--output-dir', str(output)],
            cwd=self.base, env=env, capture_output=True, text=True,
            encoding='utf-8', timeout=10)

    def rows(self, output):
        with (output / 'planning.csv').open(encoding='utf-8', newline='') as stream:
            reader = csv.DictReader(stream)
            return reader.fieldnames, list(reader)

    def summary(self, output):
        return json.loads((output / 'summary.json').read_text(encoding='utf-8'))

    def test_mixed_stream_and_chunk_boundaries_preserve_all_fields(self):
        # Catches fragment loss, dropped fields and unsigned goal conversion.
        raw = b'noise\x00' + old_motion_frame() + planning_frame() + planning_frame(seq=8)
        self.source.write_bytes(raw)
        expected_row = dict(seq='7', flags='25', timestamp_ms='123456',
                            scan_id='42', generation='13', age_ms='275', stop_mm='620',
                            slow_mm='820', clearance_mm='80', speed_mm_s='79', cycles='301',
                            candidate='3', candidate_count='9', target_l='3', target_r='6',
                            admissible_mask='24', unknown_mask='256', goal_cd='-1200',
                            command_epoch='18', latency_budget_ms='595')
        summaries = []
        for size in (1, 7, 45, 46, 4096):
            with self.subTest(chunk_size=size):
                output = self.base / f'chunk-{size}'
                summary = self.export(self.source, output, chunk_size=size)
                names, rows = self.rows(output)
                self.assertEqual(names, [f.name for f in fields(LidarPlanningTelemetry)])
                self.assertEqual(rows, [expected_row, dict(expected_row, seq='8')])
                self.assertEqual(summary, self.summary(output))
                self.assertEqual(summary['input_sha256'], hashlib.sha256(raw).hexdigest())
                self.assertEqual(summary['input_bytes'], len(raw))
                self.assertEqual(summary['input_path'], str(self.source.resolve()))
                self.assertEqual((summary['accepted_frames'], summary['bad_frames'],
                                  summary['trailing_buffer_bytes']), (2, 0, 0))
                summaries.append(summary)
        self.assertTrue(all(s == summaries[0] for s in summaries))

    def test_duplicate_and_wrapped_timestamps_keep_order_without_live_claims(self):
        # Catches C6-style deduplication and treating wraparound as elapsed time.
        stamps = [0xfffffffe, 0xfffffffe, 2, 2, 0xfffffffe]
        self.source.write_bytes(b''.join(planning_frame(seq=i, timestamp_ms=stamp,
                                                       flags=0x203f)
                                         for i, stamp in enumerate(stamps)))
        output = self.base / 'duplicates'
        summary = self.export(self.source, output, chunk_size=1)
        self.assertEqual([int(row['timestamp_ms']) for row in self.rows(output)[1]], stamps)
        self.assertEqual(summary['accepted_frames'], 5)
        self.assertEqual(summary['consecutive_duplicate_timestamps'], 2)
        self.assertEqual((summary['first_timestamp_ms'], summary['last_timestamp_ms']),
                         (0xfffffffe, 0xfffffffe))
        self.assertEqual(summary['physical_validation'], 'not_assessed')
        self.assertEqual(summary['live_status'], 'not_assessed')
        self.assertNotIn('duration_ms', summary)
        self.assertNotIn('lost_frames', summary)

    def test_cli_valid_capture_runs_without_site_packages(self):
        # -S excludes site packages, including serial; valid offline data returns zero.
        self.source.write_bytes(old_motion_frame() + planning_frame())
        output = self.base / 'valid'
        result = self.run_cli(output)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.summary(output)['accepted_frames'], 1)

    def test_cli_bad_crc_and_semantic_rejection_preserve_next_frame(self):
        # Catches reporting only CRC errors or discarding accepted evidence after a bad frame.
        corrupt = bytearray(planning_frame())
        corrupt[10] ^= 1
        self.source.write_bytes(bytes(corrupt) + planning_frame(candidate_count=8)
                                + planning_frame(seq=9))
        output = self.base / 'bad-frames'
        result = self.run_cli(output)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn('data quality', result.stderr.lower())
        self.assertNotIn('Traceback', result.stderr)
        summary = self.summary(output)
        self.assertEqual((summary['accepted_frames'], summary['bad_frames'],
                          summary['trailing_buffer_bytes']), (1, 2, 0))
        self.assertEqual([row['seq'] for row in self.rows(output)[1]], ['9'])

    def test_cli_truncated_tail_preserves_accepted_rows(self):
        # Catches ignoring unfinished C8 data at EOF.
        self.source.write_bytes(planning_frame() + planning_frame(seq=8)[:13])
        output = self.base / 'tail'
        result = self.run_cli(output)
        self.assertEqual(result.returncode, 2, result.stderr)
        summary = self.summary(output)
        self.assertEqual((summary['accepted_frames'], summary['bad_frames'],
                          summary['trailing_buffer_bytes']), (1, 0, 13))
        self.assertEqual(len(self.rows(output)[1]), 1)

    def test_empty_old_only_and_noise_are_not_successful_c8_captures(self):
        # Catches zero-C8 success and invented initial timestamps.
        for name, raw in [('empty', b''), ('old-only', old_motion_frame()),
                          ('noise', b'not a telemetry capture')]:
            with self.subTest(name=name):
                self.source.write_bytes(raw)
                output = self.base / name
                result = self.run_cli(output)
                self.assertEqual(result.returncode, 2, result.stderr)
                summary = self.summary(output)
                self.assertEqual(summary['schema_version'], 1)
                self.assertEqual(summary['accepted_frames'], 0)
                self.assertIsNone(summary['first_timestamp_ms'])
                self.assertIsNone(summary['last_timestamp_ms'])
                self.assertEqual(summary['physical_validation'], 'not_assessed')
                self.assertEqual(summary['live_status'], 'not_assessed')
                self.assertEqual(self.rows(output)[1], [])

    def test_existing_output_directory_is_untouched(self):
        # Catches accidental overwrite or cleanup of an earlier evidence directory.
        self.source.write_bytes(planning_frame())
        output = self.base / 'existing'
        output.mkdir()
        (output / 'planning.csv').write_bytes(b'prior CSV')
        (output / 'summary.json').write_bytes(b'prior summary')
        before = {p.name: p.read_bytes() for p in output.iterdir()}
        result = self.run_cli(output)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('error', result.stderr.lower())
        self.assertNotIn('Traceback', result.stderr)
        self.assertEqual({p.name: p.read_bytes() for p in output.iterdir()}, before)
        empty = self.base / 'existing-empty'
        empty.mkdir()
        with self.assertRaises(FileExistsError):
            self.export(self.source, empty)
        self.assertEqual(list(empty.iterdir()), [])

    def test_missing_or_unreadable_input_does_not_create_output(self):
        # A directory cannot be opened as a binary input, even with elevated permissions.
        for name, source in [('missing', self.base / 'missing.bin'),
                             ('directory', self.base)]:
            with self.subTest(name=name):
                output = self.base / f'out-{name}'
                result = self.run_cli(output, source=source)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('error', result.stderr.lower())
                self.assertNotIn('Traceback', result.stderr)
                self.assertFalse(output.exists())

    def test_output_io_error_is_a_clear_nonzero_cli_error(self):
        # A regular file as parent forces a real mkdir I/O error on Windows and POSIX.
        self.source.write_bytes(planning_frame())
        blocker = self.base / 'regular-file'
        blocker.write_bytes(b'keep')
        result = self.run_cli(blocker / 'output')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('error', result.stderr.lower())
        self.assertNotIn('Traceback', result.stderr)
        self.assertEqual(blocker.read_bytes(), b'keep')

    def test_device_input_is_rejected_without_creating_output(self):
        # Opening NUL or /dev/null succeeds, but it is not an existing capture file.
        source = Path('NUL') if os.name == 'nt' else Path('/dev/null')
        output = self.base / 'device-output'
        result = self.run_cli(output, source=source)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('error', result.stderr.lower())
        self.assertFalse(output.exists())

    def test_invalid_chunk_size_is_rejected_before_output_creation(self):
        # Prevents read(0) silently treating a nonempty capture as empty, or read(-1) loading it all.
        self.source.write_bytes(planning_frame())
        for size in (0, -1):
            with self.subTest(chunk_size=size):
                output = self.base / f'invalid-{size}'
                with self.assertRaises(ValueError):
                    self.export(self.source, output, chunk_size=size)
                self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
