"""Receive-only HDMI telemetry viewer. Q exits; no UART writes."""
import curses
import time
from protocol import PosixTelemetryLink

DEVICE = '/dev/serial/by-path/platform-fc800000.usb-usb-0:1.3:1.0-port0'

def main(screen):
    curses.curs_set(0)
    screen.nodelay(True)
    link = PosixTelemetryLink(DEVICE)
    try:
        while screen.getch() not in (ord('q'), ord('Q'), 27):
            now = time.monotonic()
            link.poll(now)
            live = link.live(now)
            t = link.telemetry if live else None
            def dist(value, bit):
                return f'{value / 10:.1f} cm' if t.health & bit and value != 65535 else 'NO VALID DATA'
            lines = [
                'FPGA ROBOT - LIVE SENSOR DATA',
                'Receive only | No motor commands | Q / Esc: close',
                '',
                'Serial link: ' + ('LIVE' if live else 'WAITING / STALE'),
                f'Valid frames: {link.frames}    CRC errors: {link.crc_errors}',
                '',
                'Left distance : ' + (dist(t.distance_l, 4) if t else '--'),
                'Right distance: ' + (dist(t.distance_r, 8) if t else '--'),
                '',
                'Wheel speed (encoder counts / 20 ms)',
                f'Left: {t.velocity_l if t else "--"}     Right: {t.velocity_r if t else "--"}',
                '',
                ('Roll / Pitch / Yaw: %.2f / %.2f / %.2f deg' % (t.roll/100,t.pitch/100,t.yaw/100)) if t and t.health & 2 else 'IMU: NO VALID DATA',
                '',
                f'State code: {t.state}    Fault bits: 0x{t.fault:04X}' if t else 'State: --',
                ('Brake: %s    Armed: %s' % ('ON' if t.health & 32 else 'OFF', 'YES' if t.health & 64 else 'NO')) if t else '',
                ('Stop permission: ' + ('READY' if t.health & 16 else 'NOT READY')) if t else '',
                '',
                'FPGA SRAM firmware is lost on power-off.',
                link.error,
            ]
            screen.erase()
            height, width = screen.getmaxyx()
            for y, line in enumerate(lines[:height-1]):
                if width > 1:
                    screen.addnstr(y, 0, line, width-1, curses.A_BOLD if y == 0 else 0)
            screen.refresh()
            time.sleep(0.1)
    finally:
        link.close()

if __name__ == '__main__':
    curses.wrapper(main)
