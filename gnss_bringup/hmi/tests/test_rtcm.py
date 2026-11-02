from gnss_hmi.parsers.rtcm import RtcmFramer, crc24q, parse_1005


def _frame(payload: bytes) -> bytes:
    head = bytes([0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF])
    body = head + payload
    return body + crc24q(body).to_bytes(3, "big")


def _payload_msgtype(msg_type: int, tail_bits: int) -> bytes:
    """Payload whose first 12 bits are msg_type, rest zeros."""
    n_bytes = (12 + tail_bits + 7) // 8
    v = msg_type << (n_bytes * 8 - 12)
    return v.to_bytes(n_bytes, "big")


def _payload_1005(x: float, y: float, z: float) -> bytes:
    """152-bit type-1005 payload with given ECEF coords (meters)."""
    def enc(val: float) -> int:
        i = round(val / 1e-4)
        return i & ((1 << 38) - 1)
    bits = 0
    bits |= 1005 << (152 - 12)          # DF002 message number
    bits |= enc(x) << (152 - 34 - 38)   # ECEF-X at bit 34
    bits |= enc(y) << (152 - 74 - 38)   # ECEF-Y at bit 74
    bits |= enc(z) << (152 - 114 - 38)  # ECEF-Z at bit 114
    return bits.to_bytes(19, "big")


def test_single_frame():
    payload = _payload_msgtype(1074, tail_bits=52)
    msgs = RtcmFramer().feed(_frame(payload))
    assert len(msgs) == 1
    assert msgs[0].msg_type == 1074
    assert msgs[0].raw == _frame(payload)


def test_split_across_chunks_and_garbage_prefix():
    f = RtcmFramer()
    frame = _frame(_payload_msgtype(1124, 52))
    assert f.feed(b"\x00garbage" + frame[:5]) == []
    msgs = f.feed(frame[5:] + frame)  # remainder + a second full frame
    assert [m.msg_type for m in msgs] == [1124, 1124]


def test_crc_error_skips_byte_not_frame():
    f = RtcmFramer()
    frame = bytearray(_frame(_payload_msgtype(1005, 140)))
    frame[10] ^= 0xFF  # corrupt
    good = _frame(_payload_msgtype(1084, 52))
    msgs = f.feed(bytes(frame) + good)
    assert [m.msg_type for m in msgs] == [1084]
    assert f.crc_errors >= 1


def test_parse_1005_roundtrip():
    x, y, z = -2148744.1234, 4426641.2345, 4044655.9876
    px, py, pz = parse_1005(_payload_1005(x, y, z))
    assert abs(px - x) < 1e-4 and abs(py - y) < 1e-4 and abs(pz - z) < 1e-4
