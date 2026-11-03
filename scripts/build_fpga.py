"""Prepare or build an isolated observation FPGA input set; never accesses a board.

Adapted from the project's 2026-11-02 observation build helper.
"""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import os
import re
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def input_files(source):
    files = [p for name in ('rtl', 'vendor') for p in (source / name).rglob('*') if p.is_file()]
    files += [source / name for name in ('robot.gprj', 'robot.cst', 'robot.sdc', 'build.tcl')]
    return {p.relative_to(source).as_posix(): p for p in sorted(files)}


def find_gowin(value):
    candidate = value or os.environ.get('GOWIN_SH')
    if candidate:
        direct = Path(candidate).expanduser()
        found = str(direct) if direct.is_file() else shutil.which(candidate)
    else:
        found = shutil.which('gw_sh') or shutil.which('gw_sh.exe')
    if not found or not Path(found).is_file():
        raise ValueError('Gowin gw_sh was not found; use --gowin, GOWIN_SH, or PATH.')
    return Path(found).resolve()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile', choices=('observe', 'observe_map'))
    parser.add_argument('--attempt', default=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'),
                        help='Unique letters/digits/underscore/hyphen suffix; existing output is never reused.')
    parser.add_argument('--gowin', help='Gowin gw_sh executable; overrides GOWIN_SH and PATH.')
    parser.add_argument('--prepare-only', action='store_true',
                        help='Copy and hash inputs without locating or running Gowin.')
    args = parser.parse_args(argv)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', args.attempt):
        parser.error('attempt must contain only letters, digits, underscore, or hyphen')
    try:
        tool = None if args.prepare_only else find_gowin(args.gowin)
    except ValueError as exc:
        parser.error(str(exc))
    source = ROOT / 'robot_rebuild/fpga'
    source_files = input_files(source)
    if any(not p.is_file() for p in source_files.values()) or not any((source / 'rtl').glob('*.v')):
        parser.error('The FPGA source tree is incomplete.')
    original_hashes = {name: sha(path) for name, path in source_files.items()}
    out = ROOT / 'build' / (args.profile + '_' + args.attempt)
    if out.exists():
        parser.error('Output already exists; choose another --attempt: ' + str(out))
    fpga = out / 'fpga'
    fpga.mkdir(parents=True, exist_ok=False)
    for name, path in source_files.items():
        target = fpga / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        if sha(target) != original_hashes[name]:
            raise RuntimeError('Source changed during input copy: ' + name)
    top = fpga / 'rtl/robot_top.v'
    text = top.read_text(encoding='utf-8')
    expected = {'LIDAR_PLANNER_ENABLE': 1, 'LIDAR_CONTROL': 0,
                'LIDAR_ADVANCED_CONTROL': 0, 'LIDAR_MODEL_VALIDATED': 0,
                'LIDAR_MAP_DISPLAY': 0}
    for name, value in expected.items():
        if re.findall(r'\b' + name + r'\s*=\s*(\d+)\b', text) != [str(value)]:
            raise RuntimeError('Unexpected observation source parameter: ' + name)
    if args.profile == 'observe_map':
        text, count = re.subn(r'(\bLIDAR_MAP_DISPLAY\s*=\s*)0\b', r'\g<1>1', text, count=1)
        if count != 1:
            raise RuntimeError('Expected one display parameter override.')
        top.write_text(text, encoding='utf-8', newline='\n')
        expected['LIDAR_MAP_DISPLAY'] = 1
    inputs = {name: sha(fpga / name) for name in source_files}
    record = dict(profile=args.profile, attempt=args.attempt,
                  started_at=datetime.datetime.now().astimezone().isoformat(),
                  status='prepared' if args.prepare_only else 'running',
                  source_sha256=original_hashes, build_input_sha256=inputs,
                  parameters=expected, helper_pid=os.getpid(),
                  flash_updated=False, physical_model_validated=False,
                  native_reports_reviewed=False)
    if tool:
        record.update(tool_path=str(tool), tool_sha256=sha(tool))
    status = out / 'status.json'

    def save():
        status.write_text(json.dumps(record, indent=2) + '\n', encoding='utf-8')

    (out / 'sources.json').write_text(json.dumps(record, indent=2) + '\n', encoding='utf-8')
    save()
    print(json.dumps({'profile': args.profile, 'status': record['status'], 'output_dir': str(out)}), flush=True)
    if args.prepare_only:
        return 0
    try:
        with (out / 'build.log').open('wb') as log:
            process = subprocess.Popen([str(tool), str(fpga / 'build.tcl')], cwd=fpga,
                                       stdout=log, stderr=subprocess.STDOUT,
                                       creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            record['tool_pid'] = process.pid
            save()
            exit_code = process.wait()
        artifacts = {p.relative_to(fpga).as_posix(): dict(sha256=sha(p), bytes=p.stat().st_size)
                     for p in (fpga / 'impl').rglob('*') if p.is_file() and
                     p.suffix.lower() in ('.fs', '.rpt', '.html', '.htm', '.sdc', '.vg', '.json')}
        record.update(status='finished', exit_code=exit_code, artifacts=artifacts,
                      build_inputs_unchanged=all(sha(fpga / key) == value for key, value in inputs.items()),
                      source_inputs_still_match=all(path.is_file() and sha(path) == original_hashes[name]
                                                   for name, path in source_files.items()),
                      completed_at=datetime.datetime.now().astimezone().isoformat())
    except BaseException as exc:
        record.update(status='helper_error', error_type=type(exc).__name__)
        raise
    finally:
        save()
    print(json.dumps({key: record[key] for key in ('profile', 'status', 'exit_code')}), flush=True)
    if exit_code:
        print('Gowin failed; inspect build.log in the output directory.', flush=True)
    else:
        print('Review native setup/hold and clock reports before treating the build as timing-verified.', flush=True)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
