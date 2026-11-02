"""Tests for the replay engine (spec §6)."""
import pytest
from gnss_hmi.replay import replay_messages
from gnss_hmi.storage.epochs import Epoch, EpochStore
from gnss_hmi.storage.events import EventStore


async def _collect(gen):
    return [m async for m in gen]


@pytest.mark.asyncio
async def test_replay_reconstructs_timeline(tmp_path):
    """Main test: replay reconstructs a realistic timeline from epoch and event stores."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")
    ep.add(Epoch(t=100.2, src="can", q=4, lat=44.5, lon=90.2, heading=170.0, speed=5.0))
    ep.add(Epoch(t=101.3, src="can", q=4, lat=44.6, lon=90.3, heading=171.0, speed=5.1))
    ep.add(Epoch(t=101.5, src="rtkrcv", q=1, sats=38, lat=44.6, lon=90.3))
    rid = ev.record(100.5, "diagnosis", "open", "差分中断", level="serious", code="corr_outage")
    ev.close_event(rid, 101.8)

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 102.0, speed=10.0, sleep=nosleep))

    kinds = [m["type"] for m in msgs]
    assert kinds[-1] == "replay_end"
    assert kinds.count("position") == 2
    assert kinds.count("status") >= 2  # one per whole second
    opens = [m for m in msgs if m["type"] == "event" and m["action"] == "open"]
    closes = [m for m in msgs if m["type"] == "event" and m["action"] == "close"]
    assert opens[0]["event"]["code"] == "corr_outage" and len(closes) == 1
    st = [m for m in msgs if m["type"] == "status"][-1]
    assert st["sol"]["q"] == 1 and st["can"]["heading"] == 171.0
    # ordering: messages non-decreasing in t
    ts = [m["t"] for m in msgs]
    assert ts == sorted(ts)


@pytest.mark.asyncio
async def test_replay_empty_stores(tmp_path):
    """Empty epoch/event stores produce only status messages and replay_end."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 102.0, sleep=nosleep))

    kinds = [m["type"] for m in msgs]
    assert kinds[-1] == "replay_end"
    # Status messages for seconds 100 and 101 (ceil(100) to floor(102))
    assert kinds.count("status") == 3
    assert kinds.count("position") == 0
    assert kinds.count("event") == 0


@pytest.mark.asyncio
async def test_replay_position_message_fields(tmp_path):
    """Position messages have correct structure and fields."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")
    ep.add(Epoch(t=100.5, src="can", q=4, lat=44.5, lon=90.2, heading=170.0, speed=5.0))

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 101.0, sleep=nosleep))
    pos = [m for m in msgs if m["type"] == "position"][0]

    assert pos["type"] == "position"
    assert pos["t"] == 100.5
    assert pos["src"] == "can"
    assert pos["lat"] == 44.5
    assert pos["lon"] == 90.2
    assert pos["heading"] == 170.0
    assert pos["q"] == 4
    assert pos["speed"] == 5.0


@pytest.mark.asyncio
async def test_replay_status_message_fields(tmp_path):
    """Status messages have correct structure and snap to latest epochs."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")
    ep.add(Epoch(t=100.2, src="can", q=4, lat=44.5, lon=90.2, heading=170.0, speed=5.0))
    ep.add(Epoch(t=100.8, src="can", q=4, lat=44.6, lon=90.3, heading=171.0, speed=5.1))
    ep.add(Epoch(t=101.5, src="rtkrcv", q=1, sats=38, lat=44.7, lon=90.4))

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 102.0, sleep=nosleep))
    status_100 = [m for m in msgs if m["type"] == "status" and m["t"] == 100.0][0]
    status_101 = [m for m in msgs if m["type"] == "status" and m["t"] == 101.0][0]

    # At t=100.0, only can epoch 100.2 exists (but t > 100, so should be None for status at t=100)
    assert status_100["sol"] is None
    assert status_100["can"] is None
    assert status_100["gpchc"] is None

    # At t=101.0, can 100.8 should be used, sol should be None (101.5 > 101)
    assert status_101["can"]["lat"] == 44.6
    assert status_101["can"]["heading"] == 171.0
    assert status_101["sol"] is None

    # Check verdict and corr fields
    assert status_100["verdict"]["level"] == "info"
    assert status_100["verdict"]["code"] == "replay"
    assert status_100["verdict"]["message"] == "回放"
    assert status_100["corr"]["last_t"] is None
    assert status_100["corr"]["base_offset_m"] is None


@pytest.mark.asyncio
async def test_replay_event_fields(tmp_path):
    """Event messages have correct structure and timing."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")
    rid = ev.record(100.5, "diagnosis", "open", "差分中断", level="serious", code="corr_outage")
    ev.close_event(rid, 101.8)

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 102.0, sleep=nosleep))
    opens = [m for m in msgs if m["type"] == "event" and m["action"] == "open"]
    closes = [m for m in msgs if m["type"] == "event" and m["action"] == "close"]

    assert len(opens) == 1
    open_msg = opens[0]
    assert open_msg["type"] == "event"
    assert open_msg["t"] == 100.5
    assert open_msg["action"] == "open"
    assert open_msg["event"]["t"] == 100.5
    assert open_msg["event"]["level"] == "serious"
    assert open_msg["event"]["code"] == "corr_outage"
    assert open_msg["event"]["message"] == "差分中断"

    assert len(closes) == 1
    close_msg = closes[0]
    assert close_msg["type"] == "event"
    assert close_msg["t"] == 101.8
    assert close_msg["action"] == "close"
    assert close_msg["event"]["t"] == 101.8


@pytest.mark.asyncio
async def test_replay_filters_out_of_range(tmp_path):
    """Events and epochs outside [t0, t1] are not included in output (open events within range are included)."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")
    # Add epochs outside the range
    ep.add(Epoch(t=99.0, src="can", q=4, lat=44.5, lon=90.2, heading=170.0, speed=5.0))
    ep.add(Epoch(t=100.5, src="can", q=4, lat=44.6, lon=90.3, heading=171.0, speed=5.1))
    ep.add(Epoch(t=103.0, src="can", q=4, lat=44.7, lon=90.4, heading=172.0, speed=5.2))

    # Add events
    rid1 = ev.record(99.5, "diagnosis", "open", "事件1", level="serious", code="event1")
    ev.close_event(rid1, 99.8)  # Outside range
    rid2 = ev.record(100.5, "diagnosis", "open", "事件2", level="serious", code="event2")
    ev.close_event(rid2, 101.5)  # Both within range
    rid3 = ev.record(102.0, "diagnosis", "open", "事件3", level="serious", code="event3")
    ev.close_event(rid3, 103.0)  # Open within range, close outside

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 102.0, sleep=nosleep))

    # Only position at 100.5 should be included
    positions = [m for m in msgs if m["type"] == "position"]
    assert len(positions) == 1
    assert positions[0]["t"] == 100.5

    # event2 (fully in range) and event3 open (in range) should be included
    # event3 close is outside range so not included
    events = [m for m in msgs if m["type"] == "event"]
    assert len(events) == 3
    codes = {m["event"]["code"] for m in events}
    assert codes == {"event2", "event3"}


@pytest.mark.asyncio
async def test_replay_gpchc_source(tmp_path):
    """gpchc epochs produce position messages like can."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")
    ep.add(Epoch(t=100.5, src="gpchc", q=5, lat=44.5, lon=90.2, heading=170.0, speed=5.0))

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 101.0, sleep=nosleep))
    pos = [m for m in msgs if m["type"] == "position"]
    assert len(pos) == 1
    assert pos[0]["src"] == "gpchc"
    assert pos[0]["t"] == 100.5


@pytest.mark.asyncio
async def test_replay_sleep_injection(tmp_path):
    """Sleep is called with correct time intervals (adjusted by speed)."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")
    ep.add(Epoch(t=100.0, src="can", q=4, lat=44.5, lon=90.2, heading=170.0, speed=5.0))
    ep.add(Epoch(t=102.0, src="can", q=4, lat=44.6, lon=90.3, heading=171.0, speed=5.1))

    sleep_calls = []

    async def track_sleep(duration):
        sleep_calls.append(duration)

    msgs = await _collect(replay_messages(ep, ev, 100.0, 102.0, speed=2.0, sleep=track_sleep))

    # Should have sleep calls for gaps between timeline events
    assert len(sleep_calls) > 0
    # Sum of all sleep durations should approximately equal the total time / speed
    # With a 2-second range and speed=2.0, total sleep should be ~1 second
    total_sleep = sum(sleep_calls)
    assert 0.9 < total_sleep < 1.1


@pytest.mark.asyncio
async def test_replay_end_message(tmp_path):
    """replay_end message is always the last message with correct t value."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 105.5, sleep=nosleep))

    assert msgs[-1]["type"] == "replay_end"
    assert msgs[-1]["t"] == 105.5


@pytest.mark.asyncio
async def test_replay_close_message_for_event_opened_before_window(tmp_path):
    """An event opened before t0 but closed inside [t0, t1] must still emit
    its close message -- querying events with since=t0 at the SQL level would
    exclude the row entirely (its t < t0), silently dropping the close."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")
    rid = ev.record(90.0, "diagnosis", "open", "差分中断", level="serious", code="corr_outage")
    ev.close_event(rid, 100.5)

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 102.0, sleep=nosleep))
    events = [m for m in msgs if m["type"] == "event"]

    assert len(events) == 1
    assert events[0]["action"] == "close"
    assert events[0]["t"] == 100.5
    assert events[0]["event"]["code"] == "corr_outage"


@pytest.mark.asyncio
async def test_replay_sol_key_set_matches_realtime_contract(tmp_path):
    """status.sol must be exactly the 12-key set the realtime WS contract
    uses for sol (main.py's _diagnosis_tick sol_dict): the 11 solution fields
    plus sats_json -- not a full Epoch asdict, which would leak src/heading/
    speed. sats_json must round-trip so the replay skyplot renders like live."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")
    sats_json = '[{"sat": "G05", "az": 310.8, "el": 26.2, "snr": 45.0, "used": true}]'
    ep.add(Epoch(t=100.5, src="rtkrcv", q=1, sats=38, age=1.2, ratio=20.0,
                 lat=44.6, lon=90.3, alt=600.0, sdn=0.01, sde=0.02, sdu=0.03,
                 heading=170.0, speed=5.0, sats_json=sats_json))

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 101.0, sleep=nosleep))
    st = [m for m in msgs if m["type"] == "status"][-1]

    assert st["sol"] is not None
    assert set(st["sol"].keys()) == {
        "t", "q", "sats", "age", "ratio", "lat", "lon", "alt", "sdn", "sde", "sdu",
        "sats_json"}
    assert st["sol"]["q"] == 1 and st["sol"]["sats"] == 38
    assert st["sol"]["sats_json"] == sats_json      # skyplot data survives replay


@pytest.mark.asyncio
async def test_every_replay_message_is_tagged(tmp_path):
    """Every replay message carries replay=True so the client can drop replay
    residue still in flight after the user returns to live."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")
    ep.add(Epoch(t=100.5, src="can", q=4, lat=44.5, lon=90.2, heading=170.0, speed=5.0))
    ep.add(Epoch(t=100.5, src="rtkrcv", q=1, sats=30, lat=44.5, lon=90.2))

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 101.0, sleep=nosleep))
    assert msgs and all(m.get("replay") is True for m in msgs)
    assert any(m["type"] == "position" for m in msgs)
    assert msgs[-1]["type"] == "replay_end" and msgs[-1]["replay"] is True


@pytest.mark.asyncio
async def test_replay_status_carries_forward_epoch_before_t0(tmp_path):
    """A single can epoch written before t0 must still be visible in the
    first status snapshot (carry-forward), but must NOT produce a position
    message since it falls outside [t0, t1]."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")
    ep.add(Epoch(t=90.0, src="can", q=4, lat=44.5, lon=90.2, heading=170.0, speed=5.0))

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 101.0, sleep=nosleep))

    positions = [m for m in msgs if m["type"] == "position"]
    assert len(positions) == 0

    status_100 = [m for m in msgs if m["type"] == "status" and m["t"] == 100.0][0]
    assert status_100["can"] is not None
    assert status_100["can"]["t"] == 90.0
    assert status_100["can"]["lat"] == 44.5


@pytest.mark.asyncio
async def test_replay_excludes_non_diagnosis_events(tmp_path):
    """The events table also stores link/crash rows (etype corr_link/web/
    rtkrcv/...); replay must only reconstruct diagnosis events -- the
    realtime WS stream never emits an event message for a link transition."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")
    ev.record(100.5, "corrections_link", "disconnected", "retry in 1s")
    ev.record(100.6, "web", "crashed", "restarting")
    rid = ev.record(100.7, "diagnosis", "open", "差分中断", level="serious", code="corr_outage")
    ev.close_event(rid, 101.0)

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 102.0, sleep=nosleep))
    events = [m for m in msgs if m["type"] == "event"]

    assert len(events) == 2                    # open + close for the diagnosis row only
    codes = {m["event"]["code"] for m in events}
    assert codes == {"corr_outage"}


@pytest.mark.asyncio
async def test_replay_handles_full_48h_window_efficiently(tmp_path):
    """A clamped 48h window (the WS layer now guarantees this upper bound,
    see api.py's cmd validation) must still generate its ~172800 per-second
    status messages via merge iteration, not by pre-materializing the whole
    range and sorting it -- this should stay fast even at the window's cap."""
    import time as _time

    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")

    async def nosleep(_):
        pass

    start = _time.monotonic()
    msgs = await _collect(replay_messages(ep, ev, 0.0, 172800.0, sleep=nosleep))
    elapsed = _time.monotonic() - start

    assert elapsed < 5.0
    assert msgs[-1]["type"] == "replay_end"
    assert sum(1 for m in msgs if m["type"] == "status") == 172801


@pytest.mark.asyncio
async def test_replay_status_uses_latest_snapshot(tmp_path):
    """Status messages use the latest epoch for each source ≤ the second."""
    ep = EpochStore(tmp_path / "e.db")
    ev = EventStore(tmp_path / "e.db")
    # Add multiple can epochs; status at t=101 should use the latest one ≤ 101
    ep.add(Epoch(t=100.2, src="can", q=2, lat=44.1, lon=90.1, heading=160.0, speed=4.0))
    ep.add(Epoch(t=100.8, src="can", q=3, lat=44.2, lon=90.2, heading=165.0, speed=4.5))
    ep.add(Epoch(t=101.5, src="can", q=4, lat=44.3, lon=90.3, heading=170.0, speed=5.0))

    async def nosleep(_):
        pass

    msgs = await _collect(replay_messages(ep, ev, 100.0, 102.0, sleep=nosleep))
    status_101 = [m for m in msgs if m["type"] == "status" and m["t"] == 101.0][0]

    # At t=101, should use epoch 100.8 (latest ≤ 101), not 101.5
    assert status_101["can"]["t"] == 100.8
    assert status_101["can"]["lat"] == 44.2
    assert status_101["can"]["q"] == 3
