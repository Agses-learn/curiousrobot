import math
import struct
import pytest
from stm32_base_bridge.protocol import crc16, encode_command, TelemetryParser, decode_telemetry
from stm32_base_bridge.control import CommandGate


def fixture_frame(flags=0x12):
    # Independently specify every payload field instead of round-tripping the encoder.
    payload = struct.pack('<IB6i7h', 123456, flags, 1234, -2500, 1571, 200, 0, -300,
                          16384, -16384, 8192, 164, -328, 0, 2534)
    body = bytes([3, 1, 255, 255, 43]) + payload
    return b'\xa5\x5a' + body + struct.pack('<H', crc16(body))


def test_crc_known_vector():
    assert crc16(b'123456789') == 0x29B1


def test_command_signed_units_and_wrap():
    f = encode_command(65536, -0.2, 0.3)
    assert f[:11] == bytes.fromhex('a55a010200000438ff2c01')
    assert len(f) == 13
    assert crc16(f[2:11]) == int.from_bytes(f[11:13], 'little')
    assert struct.unpack_from('<h', encode_command(1, -0.0005, 0), 7)[0] == -1


@pytest.mark.parametrize('v,w', [(0.501, 0), (0, 3.001), (math.nan, 0), (0, math.inf)])
def test_bad_command(v, w):
    with pytest.raises(ValueError):
        encode_command(0, v, w)


def test_decode_units():
    s = decode_telemetry(fixture_frame())
    assert (s.sequence, s.timestamp_ms, s.flags) == (65535, 123456, 0x12)
    assert (s.x, s.y, s.yaw, s.vx, s.wz) == (1.234, -2.5, 1.571, 0.2, -0.3)
    assert s.accel[0] == pytest.approx(9.80665)
    assert s.gyro[0] == pytest.approx(math.radians(10))
    assert s.temperature == 25.34


def test_all_split_points():
    frame = fixture_frame()
    for i in range(1, len(frame)):
        p = TelemetryParser()
        assert p.feed(frame[:i]) == []
        assert len(p.feed(frame[i:])) == 1


def test_noise_crc_bad_version_and_merge():
    p = TelemetryParser()
    good = fixture_frame()
    broken = bytearray(good)
    broken[22] ^= 0x40
    wrong = bytearray(good)
    wrong[2] = 2
    assert len(p.feed(b'junk\xa5' + broken + wrong + good * 2)) == 2
    assert p.crc_errors >= 1 and p.format_errors >= 1


def test_bounded_noise():
    p = TelemetryParser()
    p.feed(b'x' * 100000 + b'\xa5')
    assert p.buffer == b'\xa5'
    assert len(p.feed(fixture_frame()[1:])) == 1


def test_watchdog_does_not_refresh_on_output():
    g = CommandGate(timeout=0.2)
    g.accept(0.2, 0, 1.0)
    assert g.output(1.1) == (0.2, 0)
    assert g.output(1.21) == (0, 0)
    assert g.output(1.22) == (0, 0)


def test_fault_reconnect_or_unsupported_clears_command():
    g = CommandGate()
    g.accept(0.2, 0, 1.0)
    assert g.output(1.05, ready=False) == (0, 0)
    assert g.output(1.06, ready=True) == (0, 0)
    assert not g.accept(0.2, 0, 1.1, unsupported=(1,))
    assert not g.accept(math.nan, 0, 1.1)
    assert not g.accept(0.2, 0, 1.1, ready=False)
    assert g.accept(1.0, 4.0, 2.0)
    assert g.output(2.0) == (0.5, 3.0)
