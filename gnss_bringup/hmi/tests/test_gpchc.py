from gnss_hmi.parsers.gpchc import GpchcEpoch, LineFramer, parse_gpchc

BODY = ("GPCHC,2372,113755.36,174.20,1.25,-0.80,0.12,-0.05,0.30,"
        "0.0123,-0.0045,0.9987,44.50123456,90.28765432,617.123,"
        "0.02,-0.01,0.00,0.02,39,38,42,1.2,0")


def _mk(body: str) -> str:
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"${body}*{cs:02X}"


def test_parse_valid_sentence():
    e = parse_gpchc(_mk(BODY))
    assert isinstance(e, GpchcEpoch)
    assert e.week == 2372 and abs(e.tow - 113755.36) < 1e-6
    assert abs(e.lat - 44.50123456) < 1e-9
    assert abs(e.heading - 174.20) < 1e-6
    assert e.nsv1 == 39 and e.nsv2 == 38
    # status "42": high nibble = satellite status (4 = RTK fixed + heading),
    # low nibble = system state (2 = INS/GNSS integrated), per CGI-610 manual
    assert e.sat_status == 4 and e.sys_state == 2
    assert abs(e.diff_age - 1.2) < 1e-6


def test_bad_checksum_returns_none():
    assert parse_gpchc(f"${BODY}*00") is None


def test_other_sentence_returns_none():
    assert parse_gpchc(_mk("GPGGA,1,2,3")) is None


def test_line_framer_reassembles_chunks():
    f = LineFramer()
    s = _mk(BODY) + "\r\n"
    assert f.feed(s[:10].encode()) == []
    lines = f.feed((s[10:] + s).encode())
    assert lines == [_mk(BODY), _mk(BODY)]


def test_line_framer_bounds_buffer():
    f = LineFramer(max_buf=64)
    f.feed(b"x" * 100)                        # no newline: must not grow forever
    assert f.overflows == 1
    assert f.feed(b"abc\n") == ["abc"]        # still functional afterwards
