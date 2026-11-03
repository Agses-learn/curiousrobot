"""Offline PTY simulator. No physical serial devices are opened."""
import argparse
import errno
import json
import math
import os
import pty
import select
import signal
import struct
import time
import tty

from .protocol import HEADER, crc16, PAYLOAD, round_milli


def telemetry(seq, ms, x=0.0, y=0.0, yaw=0.0, vx=0.0, wz=0.0, flags=0x12):
    gyro_z = max(-32768, min(32767, round(wz * 180.0 / math.pi * 16.4)))
    body = struct.pack('<BBHB', 3, 1, seq & 0xFFFF, 43)
    body += PAYLOAD.pack(ms & 0xFFFFFFFF, flags,
                         *(round_milli(v) for v in (x, y, yaw, vx, 0.0, wz)),
                         0, 0, 16384, 0, 0, gyro_z, 2500)
    return HEADER + body + struct.pack('<H', crc16(body))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--link', required=True, help='New symlink to a pseudo-terminal')
    ap.add_argument('--log', help='Optional received command JSON-lines log')
    args = ap.parse_args()
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    if os.path.lexists(args.link):
        raise RuntimeError('Refusing to replace existing path: ' + args.link)
    os.symlink(os.ttyname(slave), args.link)
    print('SIMULATED MCU serial port: ' + args.link, flush=True)
    log = open(args.log, 'a', buffering=1) if args.log else None
    stop = False

    def shutdown(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    buffer = bytearray()
    seq = 0
    start = last = next_sample = time.monotonic()
    last_command = -1e9
    vx = wz = x = y = yaw = 0.0
    try:
        while not stop:
            select.select([master], [], [], 0.002)
            try:
                buffer.extend(os.read(master, 4096))
            except BlockingIOError:
                pass
            while len(buffer) >= 13:
                if buffer[:4] != HEADER + b'\x01\x02' or buffer[6] != 4:
                    del buffer[0]
                    continue
                packet = buffer[:13]
                if crc16(packet[2:11]) != struct.unpack_from('<H', packet, 11)[0]:
                    del buffer[0]
                    continue
                del buffer[:13]
                linear, angular = struct.unpack_from('<hh', packet, 7)
                if abs(linear) > 500 or abs(angular) > 3000:
                    vx = wz = 0.0
                    continue
                now = time.monotonic()
                last_command = now
                vx, wz = linear / 1000.0, angular / 1000.0
                # Approximate STM32 common wheel-speed scaling and quantization.
                left, right = vx - wz * 0.185 / 2, vx + wz * 0.185 / 2
                scale = min(1.0, 0.5 / max(abs(left), abs(right), 1e-9))
                left, right = round(left * scale * 100) / 100, round(right * scale * 100) / 100
                vx, wz = (left + right) / 2, (right - left) / 0.185
                if log:
                    log.write(json.dumps({'time': now, 'linear': linear,
                                          'angular': angular}) + '\n')
            now = time.monotonic()
            if now - last_command >= 0.3:
                vx = wz = 0.0
            if now >= next_sample:
                dt = now - last
                x += vx * math.cos(yaw) * dt
                y += vx * math.sin(yaw) * dt
                yaw = (yaw + wz * dt + math.pi) % (2 * math.pi) - math.pi
                flags = 0x12 | (0x04 if now - last_command < 0.3 else 0)
                frame = telemetry(seq, int((now - start) * 1000), x, y, yaw, vx, wz, flags)
                try:
                    os.write(master, frame)
                except BlockingIOError:
                    pass
                last = now
                next_sample = now + 0.02
                seq = (seq + 1) & 0xFFFF
    finally:
        if os.path.islink(args.link) and os.readlink(args.link) == os.ttyname(slave):
            os.unlink(args.link)
        os.close(master)
        os.close(slave)
        if log:
            log.close()
