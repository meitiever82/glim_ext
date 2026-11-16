import asyncio
import json
import socket
import struct
import textwrap
import time

from gnss_hmi.config import load_config
from gnss_hmi.main import build_app
from gnss_hmi.parsers.rtcm import crc24q
from gnss_hmi.parsers.rtksol import RtkSolution
from gnss_hmi.storage.epochs import Epoch


def _cfg(tmp_path, udp_port, corr_port):
    p = tmp_path / "config.yaml"
    p.write_text(textwrap.dedent(f"""\
        data_root: {tmp_path}/log
        db_path: {tmp_path}/rtk.db
        corrections: {{host: 127.0.0.1, port: {corr_port}}}
        raw_obs: {{host: 127.0.0.1, port: 1, listen: false}}
        gnss_solution: {{host: 127.0.0.1, port: 0, listen: true}}
        can_channel: virtual:p2app
        reserve: {{corrections_port: 0, raw_obs_port: 0}}
        retention_days: 14
        disk_watermark_pct: 85.0
        publish: {{enabled: true, host: 127.0.0.1, port: {udp_port}}}
        diagnosis: {{corr_gap_s: 0.5, close_hysteresis_s: 0.5}}
        """))
    return load_config(p)


def _gpchc_line():
    body = ("GPCHC,2372,113755.36,174.20,1.25,-0.80,0.12,-0.05,0.30,"
            "0.0123,-0.0045,0.9987,44.50123456,90.28765432,617.123,"
            "0.02,-0.01,0.00,0.02,39,38,42,1.2,0")
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"${body}*{cs:02X}\r\n".encode()


async def test_epochs_and_corr_outage_event(tmp_path):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0)); rx.setblocking(False)
    # corrections server: sends one 1074 frame then goes silent -> outage
    payload = (1074 << 4).to_bytes(2, "big") + b"\x00" * 6
    head = bytes([0xD3, 0x00, len(payload)])
    frame = head + payload + crc24q(head + payload).to_bytes(3, "big")

    async def corr_handler(reader, writer):
        writer.write(frame)
        await writer.drain()
        await asyncio.sleep(30)
    corr_srv = await asyncio.start_server(corr_handler, "127.0.0.1", 0)
    corr_port = corr_srv.sockets[0].getsockname()[1]

    app = build_app(_cfg(tmp_path, rx.getsockname()[1], corr_port))
    task = asyncio.create_task(app.run_forever())
    await asyncio.sleep(0.1)
    # feed route 3 (GPCHC) via listen port
    _, w = await asyncio.open_connection("127.0.0.1", app.sol_collector_port())
    w.write(_gpchc_line()); await w.drain()
    await asyncio.sleep(2.5)          # > corr_gap_s: outage rule must fire
    task.cancel()
    await app.shutdown()

    assert app.epochs.latest("gpchc").sats == 39
    codes = [e.code for e in app.events.query() if e.etype == "diagnosis"]
    assert "corr_outage" in codes
    # event also published on UDP
    loop = asyncio.get_running_loop()
    seen = []
    try:
        while True:
            data = await asyncio.wait_for(loop.sock_recv(rx, 4096), 0.3)
            seen.append(json.loads(data.decode()))
    except asyncio.TimeoutError:
        pass
    assert any(m["type"] == "gnss_event" and m["event"] == "corr_outage" for m in seen)
    w.close(); corr_srv.close(); rx.close()


def _minimal_cfg(tmp_path, **diag_overrides):
    p = tmp_path / "config.yaml"
    diag = {"corr_gap_s": 30.0, "close_hysteresis_s": 0.5, **diag_overrides}
    diag_lines = ", ".join(f"{k}: {v}" for k, v in diag.items())
    p.write_text(textwrap.dedent(f"""\
        data_root: {tmp_path}/log
        db_path: {tmp_path}/rtk.db
        corrections: {{host: 127.0.0.1, port: 1}}
        raw_obs: {{host: 127.0.0.1, port: 1}}
        gnss_solution: {{host: 127.0.0.1, port: 0, listen: true}}
        can_channel: virtual:{tmp_path.name}
        reserve: {{corrections_port: 0, raw_obs_port: 0}}
        retention_days: 14
        disk_watermark_pct: 85.0
        rtkrcv: {{binary: "/bin/true"}}
        diagnosis: {{{diag_lines}}}
        """))
    return load_config(p)


def _fake_sol(lat=44.5, lon=90.28, q=1, ns=30):
    return RtkSolution(t=0.0, lat=lat, lon=lon, alt=600.0, q=q, ns=ns,
                       sdn=0.01, sde=0.01, sdu=0.02, age=1.0, ratio=20.0)


def test_stale_solution_degrades_to_no_solution(tmp_path):
    """Item 1: if rtkrcv stops emitting, a frozen self._sol must not keep
    reporting a frozen fix forever -- past sol_stale_s it must gate to None
    and degrade through the existing no_solution warning path."""
    app = build_app(_minimal_cfg(tmp_path, sol_stale_s=5.0))
    app._corr_last_t = time.time()
    app._sol = _fake_sol()
    app._sol_t = time.time()
    app._diagnosis_tick()                    # fresh sol -> no no_solution yet
    codes = [e.code for e in app.events.query() if e.etype == "diagnosis"]
    assert "no_solution" not in codes

    app._sol_t = time.time() - 10            # older than sol_stale_s
    app._diagnosis_tick()
    codes = [e.code for e in app.events.query() if e.etype == "diagnosis"]
    assert "no_solution" in codes
    app._bus.shutdown()
    app.events.close()


def test_rtkrcv_epochs_throttled_to_1hz(tmp_path):
    """Item 5: rtkrcv epochs must be decimated to 1Hz like gpchc/can, not
    written at the solver's full solution rate."""
    app = build_app(_minimal_cfg(tmp_path))
    line = ("2026/08/27 04:15:55.400   44.501234567   90.287654321   617.1234"
            "   1  38   0.0110   0.0123   0.0322  -0.0001   0.0002   0.0003"
            "   0.80   25.0\r\n").encode()
    app._on_rtksol(line, 100.2)
    app._on_rtksol(line, 100.7)               # same integer second -> dropped
    rows = app.epochs.query("rtkrcv", 0, 1e12)
    assert len(rows) == 1
    app._on_rtksol(line, 101.1)               # next second -> kept
    rows = app.epochs.query("rtkrcv", 0, 1e12)
    assert len(rows) == 2
    app._bus.shutdown()
    app.events.close()


def test_stale_can_epoch_gated_from_fallbacks(tmp_path):
    """A stale CAN epoch (link died while diff age was high) must not feed the
    corr_age fallback -- rule 1 would blame the corrections link for a dead
    CAN feed -- nor pin event locations to a long-gone position."""
    app = build_app(_minimal_cfg(tmp_path, corr_gap_s=1e9, age_max_s=10.0))
    now = time.time()
    app._corr_last_t = now                       # corrections link is alive
    app.epochs.add(Epoch(t=now - 60, src="can", q=4, sats=30, age=99.0,
                         lat=44.5, lon=90.28))   # minutes-old epoch, high age
    app._diagnosis_tick()
    evs = [e for e in app.events.query() if e.etype == "diagnosis"]
    assert "corr_outage" not in [e.code for e in evs]
    assert all(e.lat is None for e in evs)       # stale position not attached

    # A fresh CAN epoch must still feed the age fallback.
    app.epochs.add(Epoch(t=time.time(), src="can", q=4, sats=30, age=99.0,
                         lat=44.5, lon=90.28))
    app._diagnosis_tick()
    codes = [e.code for e in app.events.query() if e.etype == "diagnosis"]
    assert "corr_outage" in codes
    app._bus.shutdown()
    app.events.close()


def test_diagnosis_transition_broadcasts_top_level_t(tmp_path):
    """The realtime event broadcast (main.py _on_diagnosis_transition) must
    carry a top-level "t" alongside the nested event.t, so realtime and
    replay event messages are field-identical (replay.replay_messages emits
    {"type": "event", "t": ..., "action": ..., "event": {...}})."""
    from gnss_hmi.diagnosis.rules import Verdict

    app = build_app(_minimal_cfg(tmp_path))
    q = app.broadcaster.subscribe()
    v = Verdict("serious", "corr_outage", "差分中断")
    app._on_diagnosis_transition("open", v, 123.5)

    msg = q.get_nowait()
    assert msg["type"] == "event"
    assert msg["t"] == 123.5
    assert msg["action"] == "open"
    assert msg["event"]["t"] == 123.5
    assert msg["event"]["code"] == "corr_outage"
    app._bus.shutdown()
    app.events.close()


def test_status_corr_fields_are_none_when_unset(tmp_path):
    """I4: corr.last_t/base_offset_m must surface as null (never received)
    when unset, not be coerced to 0.0 -- 0.0 is a legitimate offset/timestamp
    value, so the coercion made "never received" indistinguishable from it.
    This mirrors replay.py's status.corr contract, which is already null."""
    app = build_app(_minimal_cfg(tmp_path))
    app._diagnosis_tick()
    assert app.last_status["corr"]["last_t"] is None
    assert app.last_status["corr"]["base_offset_m"] is None
    app._bus.shutdown()
    app.events.close()


def test_div_since_cleared_when_inputs_vanish(tmp_path):
    """Item 7: if divergence was held and the CAN/sol pairing drops out (no
    fresh inputs to compare), the stale _div_since timer must not survive to
    fire rule 7 instantly once data returns."""
    app = build_app(_minimal_cfg(tmp_path))
    t = time.time()
    app._sol = _fake_sol(lat=44.5, lon=90.28)
    app._sol_t = t
    app.epochs.add(Epoch(t=t, src="can", q=4, sats=30, age=1.0,
                         lat=44.50002, lon=90.28002))   # far enough to diverge
    app._diagnosis_tick()
    assert app._div_since is not None            # divergence held

    app._sol_t = t - 10                            # pairing goes stale
    app._diagnosis_tick()
    assert app._div_since is None                  # must not survive
    app._bus.shutdown()
    app.events.close()
