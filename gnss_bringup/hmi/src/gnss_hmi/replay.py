"""Rebuild the realtime WS message stream from SQLite for a time range (spec §6)."""
from __future__ import annotations

import asyncio
import dataclasses
import math

from gnss_hmi.storage.epochs import EpochStore
from gnss_hmi.storage.events import EventStore

_SRC_POS = ("can", "gpchc")
_SRC_ALL = ("rtkrcv", "can", "gpchc")

# How far back of [t0, t1] to pull epochs purely for snapshot carry-forward
# (status.sol/can/gpchc must reflect the most recent epoch known as of each
# second, even if that epoch was written before the replay window started).
_SNAPSHOT_LOOKBACK_S = 600.0


async def replay_messages(epochs: EpochStore, events: EventStore, t0: float, t1: float,
                          speed: float = 1.0, sleep=asyncio.sleep):
    """Async generator yielding messages isomorphic with the realtime WS stream.

    For each can/gpchc epoch within [t0, t1] -> position message.
    For each whole second in [t0, t1] -> status message with latest snapshots,
    carried forward from up to _SNAPSHOT_LOOKBACK_S before t0 if needed.
    For each event open/close action within [t0, t1] -> event message.
    Ends with replay_end message.

    Every message carries replay=True so the client can discard replay
    messages still in flight after the user has returned to live (otherwise a
    few late replay points draw a straight line to the live position).

    sleep can be injected for testing (e.g., async def nosleep(_): pass).
    """
    async for m in _replay_messages(epochs, events, t0, t1, speed=speed, sleep=sleep):
        yield {**m, "replay": True}


async def _replay_messages(epochs: EpochStore, events: EventStore, t0: float, t1: float,
                           speed: float = 1.0, sleep=asyncio.sleep):
    # Rows used for snapshot carry-forward (status.sol/can/gpchc) reach back
    # _SNAPSHOT_LOOKBACK_S before t0 so a status message near t0 can still
    # reflect an epoch written just before the window opened.
    snapshot_rows = {src: epochs.query(src, t0 - _SNAPSHOT_LOOKBACK_S, t1) for src in _SRC_ALL}

    # Position + event messages are bounded by real rows (unlike the
    # once-per-second status stream below, which scales with the window
    # length) -- collecting and sorting them up front is cheap even for a
    # multi-day window.
    other: list[tuple[float, dict]] = []

    # Position messages are only emitted for epochs that actually fall inside
    # the replay window -- carry-forward is a snapshot-only concept.
    for src in _SRC_POS:
        for e in snapshot_rows[src]:
            if t0 <= e.t <= t1:
                other.append((e.t, {"type": "position", "t": e.t, "src": src,
                                    "lat": e.lat, "lon": e.lon, "heading": e.heading,
                                    "q": e.q, "speed": e.speed}))

    # Event messages: query the full history (not just since=t0) so an event
    # opened before t0 but closed inside [t0, t1] still produces its close
    # message. Open/close actions are then independently window-filtered in
    # Python -- filtering "open" at the SQL level (since=t0) would silently
    # drop the close action for any event that started before the window.
    # Only "diagnosis" rows are reconstructed here: the events table also
    # stores link/crash rows (etype corrections_link/web/rtkrcv/...) that the
    # realtime WS stream never turns into an event message.
    for row in events.query(since=0.0):
        if row.etype != "diagnosis":
            continue
        edict = {"t": row.t, "level": row.level, "code": row.code, "message": row.detail}
        if t0 <= row.t <= t1:
            other.append((row.t, {"type": "event", "t": row.t, "action": "open", "event": edict}))
        if row.t_close is not None and t0 <= row.t_close <= t1:
            other.append((row.t_close, {"type": "event", "t": row.t_close, "action": "close",
                                        "event": dict(edict, t=row.t_close)}))

    other.sort(key=lambda x: x[0])

    # Status messages for each whole second in [t0, t1], with the latest
    # snapshot per source carried forward.
    #
    # snapshot_rows[src] is ORDER BY t (EpochStore.query guarantees this), and
    # the seconds we iterate are strictly increasing, so a single
    # forward-advancing index per src -- carried across seconds rather than
    # rescanned from scratch each second -- is enough to find "latest row
    # with t <= sec": each row is visited at most once across the whole loop,
    # giving O(seconds + epochs) instead of the O(seconds * epochs) a fresh
    # per-second list comprehension would cost.
    #
    # Rather than materializing one status entry per second into a combined
    # list and sorting the whole thing (a caller-controlled t1 used to make
    # this build billions of entries synchronously on the event loop before
    # any window validation existed -- see api.py's ws cmd validation, which
    # now clamps the window to <=48h), status messages are generated lazily
    # here and merge-iterated against the small `other` list: `other` is
    # already sorted, and the per-second sequence is naturally increasing, so
    # a single forward pointer into `other` (flushed before each second's
    # status) reproduces the same global time order as the old sort-then-walk
    # without ever holding the full per-second stream in memory at once.
    idx = {src: 0 for src in _SRC_ALL}
    last: dict[str, object] = {src: None for src in _SRC_ALL}
    prev = t0
    oi = 0
    n_other = len(other)

    async def _advance(t: float) -> None:
        nonlocal prev
        if t > prev:
            await sleep((t - prev) / max(speed, 0.01))
        prev = t

    for sec in range(math.ceil(t0), math.floor(t1) + 1):
        sec_f = float(sec)
        # Flush any position/event message due strictly at-or-before this
        # second before computing/yielding the second's status -- this
        # matches the tie-break order of the old stable global sort, where
        # position/event entries were appended before status entries.
        while oi < n_other and other[oi][0] <= sec_f:
            t, msg = other[oi]
            await _advance(t)
            yield msg
            oi += 1

        for src in _SRC_ALL:
            rows = snapshot_rows[src]
            i = idx[src]
            n = len(rows)
            while i < n and rows[i].t <= sec:
                last[src] = rows[i]
                i += 1
            idx[src] = i

        sol_row = last["rtkrcv"]
        sol_dict = None
        if sol_row is not None:
            # Exact 12-key contract shared with the realtime status.sol dict
            # (main.py's _diagnosis_tick): the 11 solution fields plus
            # sats_json (the persisted per-satellite sky data), so the replay
            # skyplot renders the same as live. Not the full Epoch asdict,
            # which would also leak src/heading/speed.
            sol_dict = {
                "t": sol_row.t, "q": sol_row.q, "sats": sol_row.sats,
                "age": sol_row.age, "ratio": sol_row.ratio,
                "lat": sol_row.lat, "lon": sol_row.lon, "alt": sol_row.alt,
                "sdn": sol_row.sdn, "sde": sol_row.sde, "sdu": sol_row.sdu,
                "sats_json": sol_row.sats_json,
            }
        can_dict = dataclasses.asdict(last["can"]) if last["can"] is not None else None
        gpchc_dict = dataclasses.asdict(last["gpchc"]) if last["gpchc"] is not None else None

        await _advance(sec_f)
        yield {
            "type": "status", "t": sec_f,
            "stale": False,
            "verdict": {"level": "info", "code": "replay", "message": "回放"},
            # ROS-only fields are live-only; replay still carries the same key
            # set so the WS contract stays isomorphic (test_ws_contract.py).
            "sources": {"app": None, "fused": None},
            "sky": sol_dict["sats_json"] if sol_dict else "[]",
            "corr_rate": None,
            "link": {"corr": None, "obs": None, "rtkrcv": None, "topics": {}},
            "sol": sol_dict, "can": can_dict, "gpchc": gpchc_dict,
            "corr": {"last_t": None, "base_offset_m": None}}

    # Flush any remaining position/event messages with t strictly between
    # floor(t1) and t1 (a fractional tail past the last whole-second status).
    while oi < n_other:
        t, msg = other[oi]
        await _advance(t)
        yield msg
        oi += 1

    # End with replay_end message.
    yield {"type": "replay_end", "t": t1}
