"""Export C8 records from an existing UART binary file; never opens a serial port.

Exit codes describe file handling and offline data quality, never physical
validation, live freshness, or permission to control the vehicle.
"""
import argparse
import csv
from dataclasses import fields
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if __package__ in (None, ''):
    sys.path.insert(0, str(ROOT.parent))
from robot_rebuild.rk.protocol import LidarPlanningTelemetry, LidarPlanningTelemetryParser


def export_capture(source: Path, output_dir: Path, chunk_size: int = 4096) -> dict:
    """Stream a capture into a new directory, preserving every accepted C8 frame.

    File errors propagate to the caller. If I/O fails after directory creation,
    partial output is retained and must not be treated as a completed export.
    """
    if chunk_size <= 0:
        raise ValueError('chunk_size must be positive')
    source = Path(source)
    output_dir = Path(output_dir)
    if not source.is_file():
        raise ValueError(f'input must be an existing regular binary file: {source}')
    decoder = LidarPlanningTelemetryParser()
    digest = hashlib.sha256()
    summary = dict(schema_version=1, input_path=str(source.resolve()),
                   input_sha256=None, input_bytes=0, accepted_frames=0,
                   bad_frames=0, trailing_buffer_bytes=0,
                   first_timestamp_ms=None, last_timestamp_ms=None,
                   consecutive_duplicate_timestamps=0,
                   physical_validation='not_assessed', live_status='not_assessed')
    names = [field.name for field in fields(LidarPlanningTelemetry)]
    # Opening and first reading the input precede any output creation.
    with source.open('rb') as stream:
        data = stream.read(chunk_size)
        output_dir.mkdir(exist_ok=False)
        with (output_dir / 'planning.csv').open('x', encoding='utf-8', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=names)
            writer.writeheader()
            while data:
                digest.update(data)
                summary['input_bytes'] += len(data)
                for frame in decoder.feed(data):
                    writer.writerow(vars(frame))
                    if summary['accepted_frames'] == 0:
                        summary['first_timestamp_ms'] = frame.timestamp_ms
                    elif frame.timestamp_ms == summary['last_timestamp_ms']:
                        summary['consecutive_duplicate_timestamps'] += 1
                    summary['last_timestamp_ms'] = frame.timestamp_ms
                    summary['accepted_frames'] += 1
                data = stream.read(chunk_size)
    summary['input_sha256'] = digest.hexdigest()
    summary['bad_frames'] = decoder.bad_frames
    summary['trailing_buffer_bytes'] = len(decoder.buffer)
    with (output_dir / 'summary.json').open('x', encoding='utf-8') as json_file:
        json.dump(summary, json_file, ensure_ascii=False, indent=2)
        json_file.write('\n')
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--from-binary', type=Path, required=True,
                        help='existing UART binary capture (offline only)')
    parser.add_argument('--output-dir', type=Path, required=True,
                        help='new directory; its parent must already exist')
    args = parser.parse_args(argv)
    try:
        summary = export_capture(args.from_binary, args.output_dir)
    except (OSError, ValueError) as exc:
        print(f'Export error: {exc}', file=sys.stderr)
        return 1
    print(f"Offline C8 export: {summary['accepted_frames']} accepted frames; "
          f"{summary['bad_frames']} rejected frames; "
          f"{summary['trailing_buffer_bytes']} trailing buffer bytes; {args.output_dir}")
    print('Physical validation: not_assessed; live status: not_assessed.')
    if (summary['accepted_frames'] == 0 or summary['bad_frames'] != 0
            or summary['trailing_buffer_bytes'] != 0):
        print('Offline data quality check did not pass; CSV and summary were preserved.',
              file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
