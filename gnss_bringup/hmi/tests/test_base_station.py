from gnss_hmi.diagnosis.base_station import BaseStationMonitor
from gnss_hmi.storage.epochs import EpochStore

XYZ = (-2148744.1000, 4426641.2000, 4044655.9000)


def test_learns_baseline_then_reports_offset(tmp_path):
    store = EpochStore(tmp_path / "e.db")
    m = BaseStationMonitor(store, warmup_s=100.0)
    assert m.feed(0.0, *XYZ) is None
    assert m.feed(50.0, XYZ[0] + 0.001, XYZ[1], XYZ[2]) is None    # still warming up
    off = m.feed(101.0, *XYZ)                                       # warmup elapsed
    assert off is not None and off < 0.002                          # ~median
    assert store.kv_get("base_xyz") is not None
    off = m.feed(102.0, XYZ[0] + 0.5, XYZ[1], XYZ[2])
    assert abs(off - 0.5) < 0.01


def test_baseline_persists_across_restart(tmp_path):
    store = EpochStore(tmp_path / "e.db")
    m = BaseStationMonitor(store, warmup_s=1.0)
    m.feed(0.0, *XYZ)
    m.feed(2.0, *XYZ)                       # baseline set
    store2 = EpochStore(tmp_path / "e.db")
    m2 = BaseStationMonitor(store2, warmup_s=1.0)
    assert m2.feed(10.0, *XYZ) is not None  # no re-warmup


def test_history_records_changes_only(tmp_path):
    store = EpochStore(tmp_path / "e.db")
    m = BaseStationMonitor(store, warmup_s=1.0)
    m.feed(0.0, *XYZ)
    m.feed(2.0, *XYZ)          # same coords: history has the first sighting only
    m.feed(3.0, XYZ[0] + 0.5, XYZ[1], XYZ[2])
    hist = store.base_history()
    assert len(hist) == 2


def test_reset_updates_baseline(tmp_path):
    store = EpochStore(tmp_path / "e.db")
    m = BaseStationMonitor(store, warmup_s=1.0)
    m.feed(0.0, *XYZ); m.feed(2.0, *XYZ)
    m.reset(5.0, XYZ[0] + 0.5, XYZ[1], XYZ[2])
    assert m.feed(6.0, XYZ[0] + 0.5, XYZ[1], XYZ[2]) < 0.01


def test_corrupt_kv_falls_back_to_rewarmup(tmp_path):
    """Item 6: an unguarded float() on a corrupt stored base_xyz (e.g. a
    truncated write from a power loss under docker restart:always) must not
    raise and brick startup -- fall back to baseline=None and re-warm up."""
    store = EpochStore(tmp_path / "e.db")
    store.kv_set("base_xyz", "garbage")
    m = BaseStationMonitor(store, warmup_s=1.0)   # must not raise
    assert m.feed(0.0, *XYZ) is None              # warming up, no baseline yet
