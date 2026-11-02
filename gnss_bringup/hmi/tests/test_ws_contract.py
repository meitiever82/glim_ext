"""CONTRACT test -- the freeze point for the WS message shapes before the
Plan 3b frontend lands.

Realtime (main.py's _on_can/_diagnosis_tick/_on_diagnosis_transition) and
replay (replay.py's replay_messages) build their position/status/event
messages independently. Field *values* are allowed to differ deliberately
(see docs/ws-contract.md: replay's verdict is always
{"level":"info","code":"replay"}, its corr fields are always null, etc) --
but the *key sets* must match exactly, or the Plan 3b frontend would need
per-source-branching just to render a message. This test drives both real
builders (not hand-copied literals) and compares their sorted key sets.
"""
import struct
import textwrap

from gnss_hmi.config import load_config
from gnss_hmi.diagnosis.rules import Verdict
from gnss_hmi.main import build_app
from gnss_hmi.replay import replay_messages
from gnss_hmi.storage.epochs import Epoch, EpochStore
from gnss_hmi.storage.events import EventStore


def _cfg(tmp_path, name):
    p = tmp_path / "config.yaml"
    p.write_text(textwrap.dedent(f"""\
        data_root: {tmp_path}/log
        db_path: {tmp_path}/rtk.db
        corrections: {{host: 127.0.0.1, port: 1}}
        raw_obs: {{host: 127.0.0.1, port: 1}}
        gnss_solution: {{host: 127.0.0.1, port: 0, listen: true}}
        can_channel: virtual:{name}
        reserve: {{corrections_port: 0, raw_obs_port: 0}}
        retention_days: 14
        disk_watermark_pct: 85.0
        """))
    return load_config(p)


def _three_s20(a: float, b: float, c: float, scale: float) -> bytes:
    def enc(v: float) -> int:
        return round(v / scale) & 0xFFFFF
    u = enc(a) | (enc(b) << 20) | (enc(c) << 40)
    return u.to_bytes(8, "little")


def _cycle_frames():
    """One complete CGI-610 50Hz CAN cycle (same fixture as
    test_cgi610_can.py's test_complete_cycle_emitted_on_next_320)."""
    yield 0x320, struct.pack("<HI", 2372, 113755360) + b"\x00\x00"
    yield 0x321, _three_s20(1.23, -0.50, 0.07, 0.01)
    yield 0x322, _three_s20(0.0123, -0.0045, 0.9987, 0.0001)
    yield 0x323, bytes([2, 39, 4, 38]) + struct.pack("<H", 120) + bytes([40, 41])
    yield 0x325, struct.pack("<i", 617123) + b"\x00" * 4
    yield 0x326, _three_s20(0.0112, 0.0108, 0.0322, 0.0001)
    yield 0x327, struct.pack("<4h", 210, -15, 3, 211)
    yield 0x328, struct.pack("<4H", 25, 24, 60, 26)
    yield 0x329, _three_s20(0.01, -0.02, 0.001, 0.0001)
    yield 0x32A, struct.pack("<H", 17420) + struct.pack("<2h", 125, -80) + b"\x00\x00"
    yield 0x32B, _three_s20(0.1115, 0.05, 0.05, 0.0001)
    yield 0x32C, _three_s20(0.5, -0.2, 1.1, 0.01)
    yield 0x32D, struct.pack("<q", round(90.28765432 / 1e-8))
    yield 0x32E, struct.pack("<q", round(44.50123456 / 1e-8))


async def test_status_key_set_isomorphic(tmp_path):
    app = build_app(_cfg(tmp_path, "wscstatus"))
    app._diagnosis_tick()
    live_status = app.last_status
    app._bus.shutdown()
    app.events.close()

    ep = EpochStore(tmp_path / "r.db")
    ev = EventStore(tmp_path / "r.db")

    async def nosleep(_):
        pass
    msgs = [m async for m in replay_messages(ep, ev, 0.0, 1.0, sleep=nosleep)]
    replay_status = next(m for m in msgs if m["type"] == "status")

    assert set(live_status.keys()) == set(replay_status.keys())
    assert set(live_status["verdict"].keys()) == set(replay_status["verdict"].keys())
    assert set(live_status["corr"].keys()) == set(replay_status["corr"].keys())
    # Neither side has sol/can/gpchc data yet in this scenario -- confirms
    # the "no data" shape (None) matches on both sides too.
    for field in ("sol", "can", "gpchc"):
        assert live_status[field] is None and replay_status[field] is None


async def test_position_key_set_isomorphic(tmp_path):
    """Live: main.py's _on_can publishes a position dict once a full CGI-610
    cycle assembles. Replay: replay.py reconstructs one from the epoch that
    same cycle would have written to storage."""
    app = build_app(_cfg(tmp_path, "wscpos"))
    q = app.broadcaster.subscribe()
    for cid, data in _cycle_frames():
        app._on_can(cid, data, 100.0)
    # One more 0x320 closes the cycle and triggers the publish.
    app._on_can(0x320, struct.pack("<HI", 2372, 113755380) + b"\x00\x00", 100.02)
    live_pos = None
    while not q.empty():
        m = q.get_nowait()
        if m["type"] == "position":
            live_pos = m
    assert live_pos is not None, "expected _on_can to publish a position message"
    app._bus.shutdown()
    app.events.close()

    ep = EpochStore(tmp_path / "r.db")
    ev = EventStore(tmp_path / "r.db")
    ep.add(Epoch(t=100.0, src="can", q=4, lat=44.5, lon=90.2, heading=170.0, speed=5.0))

    async def nosleep(_):
        pass
    msgs = [m async for m in replay_messages(ep, ev, 99.0, 101.0, sleep=nosleep)]
    replay_pos = next(m for m in msgs if m["type"] == "position")

    assert set(live_pos.keys()) == set(replay_pos.keys())


async def test_event_key_set_isomorphic(tmp_path):
    app = build_app(_cfg(tmp_path, "wscevent"))
    q = app.broadcaster.subscribe()
    v = Verdict("serious", "corr_outage", "差分中断")
    app._on_diagnosis_transition("open", v, 123.5)
    live_event = q.get_nowait()
    app._bus.shutdown()
    app.events.close()

    ep = EpochStore(tmp_path / "r.db")
    ev = EventStore(tmp_path / "r.db")
    ev.record(100.5, "diagnosis", "open", "差分中断", level="serious", code="corr_outage")

    async def nosleep(_):
        pass
    msgs = [m async for m in replay_messages(ep, ev, 100.0, 101.0, sleep=nosleep)]
    replay_event = next(m for m in msgs if m["type"] == "event")

    assert set(live_event.keys()) == set(replay_event.keys())
    assert set(live_event["event"].keys()) == set(replay_event["event"].keys())
