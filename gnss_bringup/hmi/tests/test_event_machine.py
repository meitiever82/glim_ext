import pytest

from gnss_hmi.diagnosis.events import EventMachine
from gnss_hmi.diagnosis.rules import Verdict
from gnss_hmi.storage.events import EventStore

OK = Verdict("ok", "rtk_fixed", "RTK 固定")
OUT = Verdict("serious", "corr_outage", "差分中断 5s")
FLOAT_ = Verdict("warning", "ambiguity", "模糊度无法固定")
INFO = Verdict("info", "not_fixed", "非固定解（Q=2）")


def _machine(tmp_path, transitions):
    store = EventStore(tmp_path / "e.db")
    m = EventMachine(store, close_hysteresis_s=10.0,
                     on_transition=lambda kind, v, t: transitions.append((kind, v.code, t)))
    return store, m


def test_open_close_with_hysteresis(tmp_path):
    tr = []
    store, m = _machine(tmp_path, tr)
    m.update(100.0, OK)
    m.update(101.0, OUT, lat=44.5, lon=90.2)
    m.update(105.0, OUT)
    m.update(106.0, OK)               # recovery starts
    m.update(110.0, OK)               # only 4 s ok — still open
    rows = store.query()
    assert len(rows) == 1 and rows[0].state == "open" and rows[0].code == "corr_outage"
    m.update(117.0, OK)               # 11 s ok — close
    rows = store.query()
    assert rows[0].state == "closed" and rows[0].t_close == 117.0
    assert tr == [("open", "corr_outage", 101.0), ("close", "corr_outage", 117.0)]


def test_code_change_closes_and_opens(tmp_path):
    tr = []
    store, m = _machine(tmp_path, tr)
    m.update(100.0, OUT)
    m.update(101.0, FLOAT_)
    rows = store.query()
    assert [r.code for r in rows] == ["corr_outage", "ambiguity"]
    assert rows[0].state == "closed" and rows[1].state == "open"


def test_relapse_resets_hysteresis(tmp_path):
    tr = []
    store, m = _machine(tmp_path, tr)
    m.update(100.0, OUT)
    m.update(101.0, OK)
    m.update(105.0, OUT)              # relapse before hysteresis elapsed
    m.update(120.0, OK)
    m.update(131.0, OK)               # 11 s after last ok start → close
    rows = store.query()
    assert len(rows) == 1 and rows[0].state == "closed"


def test_info_does_not_open(tmp_path):
    tr = []
    store, m = _machine(tmp_path, tr)
    m.update(100.0, INFO)
    assert store.query() == [] and tr == []


def test_close_callback_exception_safety(tmp_path):
    """Verify _close() resets state before calling callback, so raises don't leave machine inconsistent."""
    store = EventStore(tmp_path / "e.db")

    def raising_callback(kind, verdict, t):
        if kind == "close":
            raise RuntimeError("Simulated callback failure")

    m = EventMachine(store, close_hysteresis_s=10.0, on_transition=raising_callback)

    # Open an event
    m.update(100.0, OUT)
    rows = store.query()
    assert len(rows) == 1 and rows[0].state == "open"

    # Try to close; callback will raise, but state should already be reset
    m.update(101.0, OK)
    with pytest.raises(RuntimeError, match="Simulated callback failure"):
        m.update(112.0, OK)  # 11 s ok — triggers close with raising callback

    # Machine state should be clean, allowing a new event of the same code
    m.update(114.0, OUT)
    rows = store.query()

    # Should have two rows: first closed (from before the exception), second open
    assert len(rows) == 2
    assert rows[0].state == "closed" and rows[0].code == "corr_outage"
    assert rows[1].state == "open" and rows[1].code == "corr_outage"


def test_peak_metrics_accumulate_and_persist(tmp_path):
    store = EventStore(tmp_path / "e.db")
    m = EventMachine(store, close_hysteresis_s=1.0)
    m.update(100.0, OUT, lat=44.0, lon=90.0, metrics={"corr_gap_s": 5.0})
    m.update(101.0, OUT, lat=44.1, lon=90.1, metrics={"corr_gap_s": 12.0})
    m.update(102.0, OK, metrics={"corr_gap_s": 0.0})
    m.update(104.0, OK)
    row = store.query()[0]
    import json
    assert row.state == "closed"
    assert json.loads(row.peak)["corr_gap_s"] == 12.0
    assert row.lat_close == 44.1 and row.lon_close == 90.1

def test_min_suffix_metrics_aggregate_min(tmp_path):
    store = EventStore(tmp_path / "e.db")
    m = EventMachine(store, close_hysteresis_s=1.0)
    m.update(100.0, OUT, metrics={"sats_min": 12.0, "corr_gap_s": 3.0})
    m.update(101.0, OUT, metrics={"sats_min": 4.0, "corr_gap_s": 9.0})
    m.update(102.0, OUT, metrics={"sats_min": 8.0, "corr_gap_s": 5.0})
    m.update(103.0, OK); m.update(105.0, OK)
    import json
    peak = json.loads(store.query()[0].peak)
    assert peak["sats_min"] == 4.0          # min, not abs-max
    assert peak["corr_gap_s"] == 9.0        # abs-max unchanged
