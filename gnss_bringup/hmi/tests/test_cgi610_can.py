import struct

from gnss_hmi.parsers.cgi610_can import Cgi610Assembler


def _three_s20(a: float, b: float, c: float, scale: float) -> bytes:
    def enc(v: float) -> int:
        return round(v / scale) & 0xFFFFF
    u = enc(a) | (enc(b) << 20) | (enc(c) << 40)
    return u.to_bytes(8, "little")


def _cycle_frames():
    """One complete 50 Hz cycle as (can_id, data) pairs."""
    yield 0x320, struct.pack("<HI", 2372, 113755360) + b"\x00\x00"   # week, tow*1e3
    yield 0x321, _three_s20(1.23, -0.50, 0.07, 0.01)                  # gyro dps
    yield 0x322, _three_s20(0.0123, -0.0045, 0.9987, 0.0001)          # accel g
    yield 0x323, bytes([2, 39, 4, 38]) + struct.pack("<H", 120) + bytes([40, 41])
    yield 0x325, struct.pack("<i", 617123) + b"\x00" * 4              # alt mm
    yield 0x326, _three_s20(0.0112, 0.0108, 0.0322, 0.0001)           # pos sigma
    yield 0x327, struct.pack("<4h", 210, -15, 3, 211)                 # vel cm/s
    yield 0x328, struct.pack("<4H", 25, 24, 60, 26)                   # vel sigma mm/s
    yield 0x329, _three_s20(0.01, -0.02, 0.001, 0.0001)               # veh accel g
    yield 0x32A, struct.pack("<H", 17420) + struct.pack("<2h", 125, -80) + b"\x00\x00"
    yield 0x32B, _three_s20(0.1115, 0.05, 0.05, 0.0001)               # att sigma
    yield 0x32C, _three_s20(0.5, -0.2, 1.1, 0.01)                     # ang rate dps
    yield 0x32D, struct.pack("<q", round(90.28765432 / 1e-8))         # lon
    yield 0x32E, struct.pack("<q", round(44.50123456 / 1e-8))         # lat


def test_complete_cycle_emitted_on_next_320():
    asm = Cgi610Assembler()
    for cid, data in _cycle_frames():
        assert asm.feed(cid, data, host_time=100.0) is None
    cyc = asm.feed(0x320, struct.pack("<HI", 2372, 113755380) + b"\x00\x00", 100.02)
    assert cyc is not None
    assert cyc.week == 2372 and abs(cyc.tow - 113755.360) < 1e-6
    assert abs(cyc.lat - 44.50123456) < 1e-8 and abs(cyc.lon - 90.28765432) < 1e-8
    assert abs(cyc.alt - 617.123) < 1e-6
    assert cyc.sys_state == 2 and cyc.sat_status == 4
    assert cyc.sats_used == 39 and abs(cyc.diff_age - 1.2) < 1e-6
    assert abs(cyc.heading - 174.20) < 1e-6 and abs(cyc.pitch - 1.25) < 1e-6
    assert abs(cyc.gyro[0] - 1.23) < 1e-6 and abs(cyc.accel[2] - 0.9987) < 1e-6
    assert abs(cyc.pos_sigma[2] - 0.0322) < 1e-6
    assert abs(cyc.vel[0] - 2.10) < 1e-6
    assert cyc.host_time == 100.0


def test_incomplete_cycle_dropped():
    asm = Cgi610Assembler()
    frames = list(_cycle_frames())[:5]          # missing most IDs
    for cid, data in frames:
        asm.feed(cid, data, 100.0)
    assert asm.feed(0x320, frames[0][1], 100.02) is None
    assert asm.incomplete == 1
