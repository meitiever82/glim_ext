from gnss_hmi.storage.events import EventStore


def test_record_and_query(tmp_path):
    s = EventStore(tmp_path / "e.db")
    rid = s.record(100.0, "corrections_link", "disconnected", "retry in 1s")
    assert rid >= 1
    s.record(101.0, "corrections_link", "connected")
    rows = s.query()
    assert len(rows) == 2
    assert rows[0].etype == "corrections_link" and rows[0].state == "disconnected"
    assert s.query(since=100.5)[0].state == "connected"
    s.close()


def test_reopen_persists(tmp_path):
    p = tmp_path / "e.db"
    s = EventStore(p)
    s.record(1.0, "x", "open")
    s.close()
    assert len(EventStore(p).query()) == 1


def test_extended_columns_and_close(tmp_path):
    s = EventStore(tmp_path / "e.db")
    rid = s.record(100.0, "diagnosis", "open", "差分中断 12s",
                   level="serious", code="corr_outage", lat=44.5, lon=90.28)
    s.close_event(rid, 130.0)
    row = s.query()[0]
    assert row.level == "serious" and row.code == "corr_outage"
    assert row.t_close == 130.0 and row.lat == 44.5


def test_migrates_old_schema(tmp_path):
    import sqlite3
    p = tmp_path / "old.db"
    db = sqlite3.connect(p)
    db.execute("CREATE TABLE events (id INTEGER PRIMARY KEY AUTOINCREMENT,"
               " t REAL NOT NULL, etype TEXT NOT NULL, state TEXT NOT NULL,"
               " detail TEXT NOT NULL DEFAULT '')")
    db.execute("INSERT INTO events (t, etype, state) VALUES (1.0, 'x', 'open')")
    db.commit(); db.close()
    s = EventStore(p)                      # must not raise; must migrate
    rows = s.query()
    assert rows[0].etype == "x" and rows[0].level is None


def test_prune_keeps_open_events(tmp_path):
    s = EventStore(tmp_path / "e.db")
    rid = s.record(100.0, "diagnosis", "open", "x")
    s.record(100.0, "diagnosis", "open", "y")
    s.close_event(rid, 110.0)
    assert s.prune(before_t=200.0) == 1          # only the closed one
    assert [r.state for r in s.query()] == ["open"]


def test_prune_deletes_non_open_link_rows(tmp_path):
    """I3: the events table also holds link/crash rows (state connected/
    disconnected/crashed/...) which the old 'state=closed' filter never
    pruned. Any non-open row past the cutoff must go; open rows (of any
    etype) must survive."""
    s = EventStore(tmp_path / "e.db")
    s.record(50.0, "corrections_link", "disconnected", "retry in 1s")
    s.record(50.0, "diagnosis", "open", "still going")
    assert s.prune(before_t=200.0) == 1
    states = [(r.etype, r.state) for r in s.query()]
    assert states == [("diagnosis", "open")]


def test_close_event_with_position_and_peak(tmp_path):
    s = EventStore(tmp_path / "e.db")
    rid = s.record(100.0, "diagnosis", "open", "x", code="corr_outage")
    s.close_event(rid, 130.0, lat=44.5, lon=90.2, peak='{"corr_gap_s": 12.0}')
    row = s.query()[0]
    assert row.lat_close == 44.5 and '"corr_gap_s"' in row.peak
