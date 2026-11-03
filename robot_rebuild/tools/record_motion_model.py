"""Read-only CSV recorder for 50Hz FPGA motion frames. Never sends serial bytes.

Use only one serial reader. On Windows move DEBUG to the PC first; the RK viewer
must not concurrently own that UART. Recording itself never starts the vehicle.
"""
import argparse
import csv
from dataclasses import fields
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'rk'),str(ROOT/'tests/deps')]
from protocol import MotionTelemetry,MotionTelemetryParser


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sources=parser.add_mutually_exclusive_group(required=True)
    sources.add_argument('--port',help='exclusive serial device, for example COM13')
    sources.add_argument('--from-binary',type=Path,help='convert an existing read-only UART capture offline')
    parser.add_argument('--seconds',type=float,default=30)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 0<args.seconds<=300:parser.error('seconds must be in (0,300]')
    decoder=MotionTelemetryParser();count=0;previous=None
    # Exclusive output prevents accidentally overwriting a useful calibration.
    with args.output.open('x',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=[field.name for field in fields(MotionTelemetry)])
        writer.writeheader()
        def consume(data):
            nonlocal count,previous
            for frame in decoder.feed(data):
                if frame.timestamp_ms==previous:continue
                writer.writerow(vars(frame));count+=1;previous=frame.timestamp_ms
        if args.from_binary:consume(args.from_binary.read_bytes())
        else:
            import serial
            with serial.Serial(args.port,115200,timeout=.1,exclusive=True) as port:
                port.reset_input_buffer();deadline=time.monotonic()+args.seconds
                while time.monotonic()<deadline:consume(port.read(4096))
    print(f'Read-only capture: {count} samples, {decoder.bad_frames} CRC/framing errors, {args.output}')
    if count<61 or decoder.bad_frames:raise SystemExit('Capture is not sufficient for model fitting; original CSV was preserved')


if __name__=='__main__':main()
