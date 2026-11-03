"""STM32 command v1 (13 bytes) and telemetry v3 (52 bytes), little endian."""
from dataclasses import dataclass
import binascii
import math
import struct

HEADER = b'\xa5\x5a'
COMMAND_SIZE = 13
TELEMETRY_SIZE = 52
IMU_VALID = 0x02
COMMAND_ACTIVE = 0x04
MOTOR_FAULT = 0x08
ODOM_VALID = 0x10
PAYLOAD = struct.Struct('<IB6i7h')


def crc16(data):
    return binascii.crc_hqx(data, 0xFFFF)


def round_milli(value):
    return int(math.copysign(math.floor(abs(value) * 1000.0 + 0.5), value))


def encode_command(sequence, linear_x, angular_z):
    if not all(math.isfinite(v) for v in (linear_x, angular_z)):
        raise ValueError('Non-finite command')
    if abs(linear_x) > 0.5 or abs(angular_z) > 3.0:
        raise ValueError('Command exceeds firmware limits')
    body = struct.pack('<BBHBhh', 1, 2, sequence & 0xFFFF, 4,
                       round_milli(linear_x), round_milli(angular_z))
    return HEADER + body + struct.pack('<H', crc16(body))


@dataclass(frozen=True)
class Sample:
    sequence: int
    timestamp_ms: int
    flags: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    accel: tuple
    gyro: tuple
    temperature: float


def decode_telemetry(frame):
    if (len(frame) != TELEMETRY_SIZE or frame[:4] != HEADER + b'\x03\x01'
            or frame[6] != 43):
        raise ValueError('Unsupported telemetry frame')
    if crc16(frame[2:50]) != struct.unpack_from('<H', frame, 50)[0]:
        raise ValueError('Bad telemetry CRC')
    seq = struct.unpack_from('<H', frame, 4)[0]
    v = PAYLOAD.unpack_from(frame, 7)
    return Sample(seq, v[0], v[1], *(x / 1000.0 for x in v[2:8]),
                  tuple(x * 9.80665 / 16384.0 for x in v[8:11]),
                  tuple(x * math.pi / (16.4 * 180.0) for x in v[11:14]),
                  v[14] / 100.0)


class TelemetryParser:
    """Bounded stream buffer; recover from partial, merged and damaged frames."""
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.format_errors = 0
        self.discarded_bytes = 0

    def feed(self, data):
        self.buffer.extend(data)
        if len(self.buffer) > 8192:
            self.discarded_bytes += len(self.buffer) - 8192
            del self.buffer[:-8192]
        samples = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                keep = 1 if self.buffer[-1:] == HEADER[:1] else 0
                self.discarded_bytes += len(self.buffer) - keep
                self.buffer[:] = self.buffer[-1:] if keep else b''
                break
            self.discarded_bytes += start
            del self.buffer[:start]
            if len(self.buffer) < 7:
                break
            if self.buffer[2:4] != b'\x03\x01' or self.buffer[6] != 43:
                self.format_errors += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < TELEMETRY_SIZE:
                break
            try:
                samples.append(decode_telemetry(self.buffer[:TELEMETRY_SIZE]))
            except ValueError:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:TELEMETRY_SIZE]
        return samples
