import sqlite3

from gnss_hmi.tiles import TileStore
from gnss_hmi.tiles.manager import (TileManager, lonlat_to_tile, refresh_due,
                                    sha_ok, tile_range, resolve_tiles_path,
                                    maintain)


def _fake_get(url):
    return ("TILE:" + url).encode()


def _rows(path):
    db = sqlite3.connect(path)
    try:
        return db.execute("SELECT COUNT(*) FROM tiles").fetchone()[0]
    finally:
        db.close()


def _meta(path):
    db = sqlite3.connect(path)
    try:
        return dict(db.execute("SELECT name, value FROM metadata").fetchall())
    finally:
        db.close()


def test_tile_math():
    assert lonlat_to_tile(0, 0, 2) == (2, 2)
    xr, yr = tile_range((-1, -1, 1, 1), 2)
    assert list(xr) == [1, 2] and list(yr) == [1, 2]
    # clamped to the valid tile grid
    assert lonlat_to_tile(-180, 90, 1) == (0, 0)


def test_build_writes_cache_metadata_and_hash(tmp_path):
    m = TileManager(str(tmp_path), http_get=_fake_get)
    out = m.build((-1, -1, 1, 1), [2])
    assert out and out.endswith(".mbtiles")
    assert m.current.is_file()
    assert sha_ok(m.current)
    assert sha_ok(m.current) and sha_ok(__import__("pathlib").Path(out))
    assert _rows(m.current) == 4
    meta = _meta(m.current)
    assert meta["minzoom"] == "2" and meta["maxzoom"] == "2"
    assert meta["format"] == "png"
    assert meta["bbox"] == "-1.000000,-1.000000,1.000000,1.000000"
    t = TileStore(str(m.current))
    try:
        # XYZ (2,1,1) is TMS row 2
        assert t.get(2, 1, 1) == _fake_get(
            "https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/2/1/1")
    finally:
        t.close()


def test_rebuild_carries_existing_tiles_and_keeps_two_versions(tmp_path):
    m = TileManager(str(tmp_path), http_get=_fake_get)
    m.build((-1, -1, 1, 1), [2], name="a")
    m.build((-1, -1, 1, 1), [2], name="b")
    # smaller bbox only touches one zoom-2 tile, the other three must survive
    m.build((0.1, 0.1, 1, 1), [2], name="c")
    assert _rows(m.current) == 4
    versions = list((tmp_path / "versions").glob("*.mbtiles"))
    assert len(versions) == 2
    assert sha_ok(m.current)


def test_offline_first_build_returns_none_then_serves_cache(tmp_path):
    def down(url):
        raise OSError("offline")

    m = TileManager(str(tmp_path), http_get=down)
    assert m.build((-1, -1, 1, 1), [2]) is None
    assert m.active() is None

    ok = TileManager(str(tmp_path), http_get=_fake_get)
    built = ok.build((-1, -1, 1, 1), [2])
    assert built
    out = m.build((-1, -1, 1, 1), [2])   # offline -> serve the carried cache
    assert out and resolve_tiles_path(str(m.current))


def test_bad_current_falls_back_to_newest_version(tmp_path):
    m = TileManager(str(tmp_path), http_get=_fake_get)
    built = m.build((-1, -1, 1, 1), [2])
    assert built
    m.current.write_bytes(b"truncated")            # keep the stale sidecar
    resolved = resolve_tiles_path(str(m.current))
    assert resolved == built
    assert not sha_ok(m.current)


def test_build_failure_leaves_current_untouched(tmp_path):
    m = TileManager(str(tmp_path), http_get=_fake_get)
    built = m.build((-1, -1, 1, 1), [2])
    before = m.current.read_bytes()
    assert m.build((-1, -1, 1, 1), []) == m.active()   # empty zoom list -> exception path
    assert m.current.read_bytes() == before


def test_resolve_trusts_sidecarless_mbtiles(tmp_path):
    p = tmp_path / "user.mbtiles"
    db = sqlite3.connect(p)
    db.execute("CREATE TABLE tiles (zoom_level INT, tile_column INT, tile_row INT, tile_data BLOB)")
    db.commit(); db.close()
    assert resolve_tiles_path(str(p)) == str(p)
    assert resolve_tiles_path("") is None
    assert resolve_tiles_path(str(tmp_path / "missing.mbtiles")) is None


def test_refresh_due_and_maintain(tmp_path):
    state = tmp_path / "last_build"
    assert refresh_due(state, 7 * 86400, now=1000)
    state.write_text("900\n")
    assert not refresh_due(state, 7 * 86400, now=1000)
    assert not refresh_due(state, 3600, now=1000)
    assert refresh_due(state, 3600, now=5000)

    m = TileManager(str(tmp_path), http_get=_fake_get)
    assert maintain(str(tmp_path), (-1, -1, 1, 1), [2], state,
                    period_s=3600, now=1000, http_get=m._get) is None   # not due
    assert maintain(str(tmp_path), (-1, -1, 1, 1), [2], state,
                    period_s=3600, now=4000, http_get=m._get) is None   # not due
    out = maintain(str(tmp_path), (-1, -1, 1, 1), [2], state,
                   period_s=3600, now=10_000, http_get=m._get)  # due -> build + stamp
    assert out
    assert state.read_text().strip() == "10000"
    assert not refresh_due(state, 3600, now=10_001)
