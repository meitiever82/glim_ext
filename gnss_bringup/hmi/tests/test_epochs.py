from gnss_hmi.storage.epochs import Epoch, EpochStore


def test_add_latest_query(tmp_path):
    s = EpochStore(tmp_path / "e.db")
    s.add(Epoch(t=100.0, src="rtkrcv", q=1, sats=38, lat=44.5, lon=90.28,
                alt=617.1, sdn=0.011, sde=0.012, sdu=0.032, age=0.8, ratio=2.5))
    s.add(Epoch(t=101.0, src="rtkrcv", q=2, sats=35, ratio=1.8))
    s.add(Epoch(t=100.5, src="can", q=4, sats=39, heading=174.2, speed=8.3))
    latest = s.latest("rtkrcv")
    assert latest.t == 101.0 and latest.q == 2 and abs(latest.ratio - 1.8) < 1e-9
    assert s.latest("can").heading == 174.2
    assert s.latest("gpchc") is None
    rows = s.query("rtkrcv", 99.0, 100.5)
    assert len(rows) == 1 and rows[0].sats == 38 and rows[0].lat == 44.5
    s.close()


def test_kv_and_base(tmp_path):
    s = EpochStore(tmp_path / "e.db")
    assert s.kv_get("base_xyz") is None
    s.kv_set("base_xyz", "1,2,3")
    s.kv_set("base_xyz", "4,5,6")            # upsert
    assert s.kv_get("base_xyz") == "4,5,6"
    s.add_base(100.0, -2148744.1, 4426641.2, 4044655.9)
    s.add_base(200.0, -2148744.2, 4426641.2, 4044655.9)
    hist = s.base_history(since=150.0)
    assert len(hist) == 1 and hist[0][0] == 200.0
    s.close()


def test_persists_across_reopen(tmp_path):
    p = tmp_path / "e.db"
    s = EpochStore(p)
    s.add(Epoch(t=1.0, src="gpchc", q=4))
    s.close()
    assert EpochStore(p).latest("gpchc").t == 1.0


def test_prune_removes_old_rows(tmp_path):
    s = EpochStore(tmp_path / "e.db")
    s.add(Epoch(t=100.0, src="can")); s.add(Epoch(t=200.0, src="can"))
    s.add_base(100.0, 1, 2, 3); s.add_base(200.0, 1, 2, 3)
    n = s.prune(before_t=150.0)
    assert n == 2
    assert s.latest("can").t == 200.0 and len(s.base_history()) == 1


def test_query_last_returns_newest_n_ordered_ascending(tmp_path):
    """I5: /api/epochs used to materialize the full range and slice
    [-limit:] in Python -- query_last pushes the "newest N" selection into
    SQL (ORDER BY t DESC LIMIT ?) and reverses just that bounded result."""
    s = EpochStore(tmp_path / "e.db")
    for i in range(10):
        s.add(Epoch(t=float(i), src="can", lat=float(i)))
    rows = s.query_last("can", 0.0, 100.0, 3)
    assert [r.t for r in rows] == [7.0, 8.0, 9.0]
    assert [r.lat for r in rows] == [7.0, 8.0, 9.0]
    s.close()


def test_query_last_respects_time_bounds(tmp_path):
    s = EpochStore(tmp_path / "e.db")
    for i in range(10):
        s.add(Epoch(t=float(i), src="can"))
    rows = s.query_last("can", 2.0, 5.0, 100)
    assert [r.t for r in rows] == [2.0, 3.0, 4.0, 5.0]
    s.close()


def test_wal_mode(tmp_path):
    s = EpochStore(tmp_path / "e.db")
    assert s._db.execute("PRAGMA journal_mode").fetchone()[0] == "wal"
