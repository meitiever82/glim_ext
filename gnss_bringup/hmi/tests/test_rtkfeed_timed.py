"""Tests for the RTCM3 framing / GPST extraction in tools/rtkfeed_timed.py."""
import importlib.util
from pathlib import Path

_MOD_PATH = Path(__file__).resolve().parents[1] / "tools" / "rtkfeed_timed.py"
_spec = importlib.util.spec_from_file_location("rtkfeed_timed", _MOD_PATH)
rtkfeed_timed = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rtkfeed_timed)


def _msm_frame(msgnum: int, tow_ms: int) -> bytes:
    """Build a synthetic RTCM3 frame: msg(12) + refstation(12)=0 + tow(30),
    padded to 7 payload bytes, with a placeholder (unchecked) CRC."""
    value = (msgnum << 44) | (0 << 32) | (tow_ms << 2)   # 56-bit payload
    payload = value.to_bytes(7, "big")
    ln = len(payload)
    header = bytes([0xD3, (ln >> 8) & 0x03, ln & 0xFF])
    return header + payload + b"\x00\x00\x00"             # CRC not verified


def test_iter_frames_and_tow():
    tow = 123456789
    frame = _msm_frame(1077, tow)
    frames = list(rtkfeed_timed.iter_frames(frame))
    assert len(frames) == 1
    msg, fr = frames[0]
    assert msg == 1077
    assert rtkfeed_timed.gps_tow_ms(fr) == tow


def test_iter_frames_recovers_after_garbage():
    a = _msm_frame(1077, 1000)
    b = _msm_frame(1087, 2000)          # GLONASS MSM7, different msgnum
    data = b"\x00\xff" + a + b"\x11" + b   # leading + interstitial garbage
    msgs = [m for m, _ in rtkfeed_timed.iter_frames(data)]
    assert msgs == [1077, 1087]


def test_build_schedule_tags_ephemeris_with_current_epoch(tmp_path):
    # obs epoch T, then an ephemeris-like frame (msg 1019, no own tow): both
    # inherit T so they release together on the schedule.
    obs = _msm_frame(1077, 500000)
    eph = _msm_frame(1019, 999999)      # 1019 isn't 1077, so its tow is ignored
    p = tmp_path / "s.rtcm3"
    p.write_bytes(obs + eph)
    sched = rtkfeed_timed.build_schedule(str(p))
    assert [t for t, _ in sched] == [500000, 500000]


def test_build_schedule_leading_frames_get_first_tow(tmp_path):
    # a frame before any 1077 (tow=None) is backfilled with the first known tow
    eph = _msm_frame(1019, 111111)
    obs = _msm_frame(1077, 700000)
    p = tmp_path / "s.rtcm3"
    p.write_bytes(eph + obs)
    sched = rtkfeed_timed.build_schedule(str(p))
    assert [t for t, _ in sched] == [700000, 700000]
