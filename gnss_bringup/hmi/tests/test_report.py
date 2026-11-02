"""Tests for report statistics."""
from gnss_hmi.config import ControlPoint
from gnss_hmi.report import compute_report
from gnss_hmi.storage.epochs import Epoch, EpochStore
from gnss_hmi.storage.events import EventStore

_DEG_PER_M = 1.0 / 111000.0


def test_report_abs_ref_control_point_deviation(tmp_path):
    """Absolute-baseline verification (§6 必含项): epochs near a surveyed control
    point contribute a deviation series; epochs far from all points are ignored."""
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    base = 1000.0
    ep.add(Epoch(t=base + 1, src="rtkrcv", q=1, lat=44.5 + 0.5 * _DEG_PER_M, lon=90.28))
    ep.add(Epoch(t=base + 2, src="rtkrcv", q=1, lat=44.5, lon=90.28))          # ~0 m
    ep.add(Epoch(t=base + 3, src="rtkrcv", q=1, lat=44.6, lon=90.28))          # ~11 km, ignored
    cps = [ControlPoint("CP1", 44.5, 90.28, 0.0)]
    r = compute_report(ep, ev, base, base + 10, control_points=cps, abs_ref_radius_m=3.0)
    assert len(r["abs_ref"]) == 2
    assert all(x["cp"] == "CP1" for x in r["abs_ref"])
    assert abs(r["abs_ref_max_m"] - 0.5) < 0.05


def test_report_abs_ref_empty_without_control_points(tmp_path):
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    ep.add(Epoch(t=1.0, src="rtkrcv", q=1, lat=44.5, lon=90.28))
    r = compute_report(ep, ev, 0.0, 10.0)
    assert r["abs_ref"] == [] and r["abs_ref_max_m"] is None


def test_report_abs_ref_ignores_non_fixed_epochs(tmp_path):
    """Float/single epochs near a control point are dm-level noise, not a frame
    shift — they must not enter the abs_ref series."""
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    base = 1000.0
    ep.add(Epoch(t=base + 1, src="rtkrcv", q=2, lat=44.5 + 0.5 * _DEG_PER_M, lon=90.28))  # float
    ep.add(Epoch(t=base + 2, src="rtkrcv", q=1, lat=44.5 + 0.3 * _DEG_PER_M, lon=90.28))  # fixed
    cps = [ControlPoint("CP1", 44.5, 90.28, 0.0)]
    r = compute_report(ep, ev, base, base + 10, control_points=cps, abs_ref_radius_m=3.0)
    assert len(r["abs_ref"]) == 1                       # only the fixed epoch
    assert abs(r["abs_ref_max_m"] - 0.3) < 0.05


def test_report_can_rtk_deviation_skips_far_apart_timestamps(tmp_path):
    """A can epoch >tolerance from the rtkrcv epoch is not paired — otherwise a
    moving vehicle's travel in the gap would masquerade as solver disagreement."""
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    base = 1000.0
    ep.add(Epoch(t=base + 1.0, src="rtkrcv", q=1, lat=44.5, lon=90.28))
    ep.add(Epoch(t=base + 1.9, src="can", q=4, lat=44.5 + 5.0 * _DEG_PER_M, lon=90.28))  # 0.9 s away
    r = compute_report(ep, ev, base, base + 10)
    assert r["can_rtk_dev"]["n"] == 0                   # no pair within tolerance


def test_report_can_rtk_deviation(tmp_path):
    """§6: 610-fused vs independent-solution deviation, matched per second."""
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    base = 1000.0
    ep.add(Epoch(t=base + 1.0, src="rtkrcv", q=1, lat=44.5, lon=90.28))
    ep.add(Epoch(t=base + 1.2, src="can", q=4, lat=44.5 + 1.0 * _DEG_PER_M, lon=90.28))  # ~1 m
    ep.add(Epoch(t=base + 2.0, src="rtkrcv", q=1, lat=44.5, lon=90.28))                  # no can pair
    r = compute_report(ep, ev, base, base + 10)
    assert r["can_rtk_dev"]["n"] == 1
    assert abs(r["can_rtk_dev"]["max_m"] - 1.0) < 0.1


def test_report_base_series_curve(tmp_path):
    """§6: base-station stability as a series (offset from first sample), not
    just a scalar max."""
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    base = 1000.0
    ep.add_base(base, -2148744.0, 4426641.0, 4044655.0)
    ep.add_base(base + 10, -2148744.3, 4426641.0, 4044655.0)   # +0.3 m
    r = compute_report(ep, ev, base, base + 100)
    assert len(r["base_series"]) == 2
    assert r["base_series"][0]["offset_m"] == 0.0
    assert abs(r["base_series"][1]["offset_m"] - 0.3) < 1e-6


def test_report_events_carry_location(tmp_path):
    """§6 problem-segment annotation: events expose their lat/lon."""
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    ev.record(1001.0, "diagnosis", "open", "遮挡", level="serious", code="low_sats",
              lat=44.5, lon=90.28)
    r = compute_report(ep, ev, 1000.0, 1100.0)
    assert r["events"][0]["lat"] == 44.5 and r["events"][0]["lon"] == 90.28


def test_report_stats(tmp_path):
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    base = 3600.0 * 100
    for i in range(10):
        ep.add(Epoch(t=base + i, src="rtkrcv", q=1 if i < 8 else 2))
    ep.add(Epoch(t=base + 3700, src="rtkrcv", q=1))
    rid = ev.record(base + 2, "diagnosis", "open", "差分中断", level="serious", code="corr_outage")
    ev.close_event(rid, base + 30)
    ep.add_base(base, -2148744.0, 4426641.0, 4044655.0)
    ep.add_base(base + 50, -2148744.5, 4426641.0, 4044655.0)

    r = compute_report(ep, ev, base, base + 7200)
    assert abs(r["fix_ratio"] - 9 / 11) < 1e-9
    assert r["hourly"][0]["epochs"] == 10 and abs(r["hourly"][0]["fix_ratio"] - 0.8) < 1e-9
    assert r["events"][0]["code"] == "corr_outage" and r["events"][0]["duration_s"] == 28.0
    assert abs(r["base_max_offset_m"] - 0.5) < 1e-6
    assert r["epoch_counts"]["rtkrcv"] == 11


def test_report_empty_range(tmp_path):
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    r = compute_report(ep, ev, 0.0, 100.0)
    assert r["fix_ratio"] is None and r["events"] == [] and r["base_max_offset_m"] is None


def test_report_event_opened_before_t0_closed_inside_window(tmp_path):
    """Event opened before t0, closed inside window → included with correct duration."""
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    base = 1000.0
    t0, t1 = base + 100, base + 200

    # Event opens before t0, closes inside window
    rid = ev.record(base + 50, "diagnosis", "open", "test outage", level="serious", code="test_code")
    ev.close_event(rid, base + 150)

    r = compute_report(ep, ev, t0, t1)
    assert len(r["events"]) == 1
    event = r["events"][0]
    assert event["code"] == "test_code"
    assert event["t"] == base + 50
    assert event["t_close"] == base + 150
    assert event["duration_s"] == 100.0

    # Verify fully-before-window event is excluded
    rid2 = ev.record(base + 10, "diagnosis", "open", "earlier outage", level="serious", code="earlier_code")
    ev.close_event(rid2, base + 40)

    r = compute_report(ep, ev, t0, t1)
    assert len(r["events"]) == 1  # Only the first event overlaps the window
    assert r["events"][0]["code"] == "test_code"


def test_report_excludes_non_diagnosis_events(tmp_path):
    """The events table also stores link/crash rows (etype corr_link/web/
    rtkrcv/...); the report's event table must only surface diagnosis rows."""
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    ev.record(50.0, "corrections_link", "disconnected", "retry in 1s")
    rid = ev.record(60.0, "diagnosis", "open", "x", level="serious", code="corr_outage")
    ev.close_event(rid, 70.0)

    r = compute_report(ep, ev, 0.0, 100.0)
    assert len(r["events"]) == 1
    assert r["events"][0]["code"] == "corr_outage"


def test_report_event_opened_before_t0_still_open(tmp_path):
    """Event opened before t0, still open (no t_close) → included with duration_s None."""
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    base = 1000.0
    t0, t1 = base + 100, base + 200

    # Event opens before t0 and remains open
    ev.record(base + 50, "diagnosis", "open", "ongoing outage", level="serious", code="ongoing_code")

    r = compute_report(ep, ev, t0, t1)
    assert len(r["events"]) == 1
    event = r["events"][0]
    assert event["code"] == "ongoing_code"
    assert event["t"] == base + 50
    assert event["t_close"] is None
    assert event["duration_s"] is None
