"""Task 8：在线 XYZ 预热 / MBTiles 缓存 / 原子切换 / 坏文件回退。

全部用假 fetch，不依赖外网。
"""
import hashlib
import json
import os
import sqlite3

from gnss_hmi.config import TilesCfg
from gnss_hmi.tiles import TileManager, TileStore, open_tile_store
from gnss_hmi.tiles.manager import planned_keys, tile_range

BBOX = (114.40, 30.50, 114.72, 30.82)     # z10 下 2x2 共 4 块
ZOOMS = (10, 10)


def _cfg(tmp_path, **kw):
    base = dict(enabled=True, source_name="esri",
                source_url="https://example.test/{z}/{y}/{x}.jpg",
                attribution="Test Imagery", cache_dir=str(tmp_path / "tiles"),
                bbox=BBOX, zoom_min=ZOOMS[0], zoom_max=ZOOMS[1],
                max_tiles=100, timeout_s=0.1)
    base.update(kw)
    return TilesCfg(**base)


class FakeFetch:
    def __init__(self, fail_keys=()):
        self.calls = []
        self.fail_keys = set(fail_keys)

    def __call__(self, url, timeout):
        self.calls.append(url)
        z, y, x = url.split("/")[-3:]
        if (int(z), int(x), int(y.split(".")[0])) in self.fail_keys:
            raise OSError("no network")
        return b"IMGDATA"


def test_tile_range_and_planned_keys():
    x0, x1, y0, y1 = tile_range(BBOX, 10)
    assert (x1 - x0 + 1, y1 - y0 + 1) == (2, 2)
    keys = planned_keys(BBOX, ZOOMS)
    assert len(keys) == 4 and len(set(keys)) == 4
    assert {(z for z, _, _ in keys)} == {10}


def test_sync_writes_mbtiles_switches_symlink_and_index(tmp_path):
    cfg = _cfg(tmp_path)
    fetch = FakeFetch()
    res = TileManager(cfg, fetch=fetch, now=lambda: 1763000000.0).sync()

    assert res["ok"] is True and res["fetched"] == 4 and res["failed"] == 0
    link = tmp_path / "tiles" / "current.mbtiles"
    assert link.is_symlink()
    assert os.readlink(link).startswith("mine_esri_") and "z10-10_wgs84.mbtiles" in os.readlink(link)

    store = TileStore(str(link))
    z, x, y = 10, 0, 0
    x0, x1, y0, y1 = tile_range(BBOX, 10)
    assert store.get(10, x0, y0) == b"IMGDATA"
    assert store.count() == 4
    store.close()

    idx = json.loads((tmp_path / "tiles" / "index.json").read_text())
    real = hashlib.sha256((tmp_path / "tiles" / os.readlink(link)).read_bytes()).hexdigest()
    assert idx["sha256"] == real and idx["tiles"] == 4
    assert idx["attribution"] == "Test Imagery" and idx["zooms"] == [10, 10]


def test_second_sync_only_fills_missing_tiles(tmp_path):
    cfg = _cfg(tmp_path)
    keys = planned_keys(BBOX, ZOOMS)
    first = FakeFetch(fail_keys=keys[1:])
    res1 = TileManager(cfg, fetch=first, now=lambda: 1.0).sync()
    assert res1["ok"] is True and res1["fetched"] == 1 and res1["failed"] == 3

    second = FakeFetch()
    res2 = TileManager(cfg, fetch=second, now=lambda: 2.0).sync()
    assert res2["fetched"] == 3 and len(second.calls) == 3        # 只补缺的 3 块
    assert TileStore(str(tmp_path / "tiles" / "current.mbtiles")).count() == 4


def test_offline_without_cache_fails_without_switching(tmp_path):
    cfg = _cfg(tmp_path)

    def boom(url, timeout):
        raise OSError("no network")

    res = TileManager(cfg, fetch=boom).sync()
    assert res["ok"] is False and "no network" not in res["reason"]
    assert not (tmp_path / "tiles" / "current.mbtiles").exists()
    assert not list((tmp_path / "tiles").glob(".build-*"))       # 临时文件已清理


def test_offline_with_cache_keeps_old_version(tmp_path):
    cfg = _cfg(tmp_path)
    TileManager(cfg, fetch=FakeFetch(), now=lambda: 1.0).sync()
    old = os.readlink(tmp_path / "tiles" / "current.mbtiles")

    def boom(url, timeout):
        raise OSError("no network")

    # 换一份 bbox 更大的配置要求补瓦片：全失败，但旧版本必须原样可用
    bigger = _cfg(tmp_path, bbox=(114.30, 30.40, 114.82, 30.92))
    res = TileManager(bigger, fetch=boom, now=lambda: 2.0).sync()
    assert res["ok"] is True and res["fetched"] == 0 and res["failed"] > 0
    assert os.readlink(tmp_path / "tiles" / "current.mbtiles") == old
    TileStore(str(tmp_path / "tiles" / "current.mbtiles")).close()


def test_corrupt_current_is_replaced(tmp_path):
    cfg = _cfg(tmp_path)
    tiles = tmp_path / "tiles"
    tiles.mkdir()
    bad = tiles / "mine_esri_20250101_z10-10_wgs84.mbtiles"
    bad.write_bytes(b"not a sqlite file at all")
    os.symlink(bad.name, tiles / "current.mbtiles")
    assert open_tile_store(str(tiles / "current.mbtiles")) is None   # 坏文件退网格

    manager = TileManager(cfg, fetch=FakeFetch(), now=lambda: 1.0)
    assert manager.info()["available"] is False
    res = manager.sync()
    assert res["ok"] is True and manager.info()["available"] is True
    assert TileStore(manager.current_path()).count() == 4


def test_prune_keeps_two_versions(tmp_path):
    cfg = _cfg(tmp_path)
    tiles = tmp_path / "tiles"
    tiles.mkdir()
    for i, name in enumerate(["mine_esri_20250101_z10-10_wgs84.mbtiles",
                              "mine_esri_20250102_z10-10_wgs84.mbtiles",
                              "mine_esri_20250103_z10-10_wgs84.mbtiles"]):
        p = tiles / name
        p.write_bytes(b"x")
        os.utime(p, (1000 + i, 1000 + i))

    TileManager(cfg, fetch=FakeFetch(), now=lambda: 1.0).sync()
    names = sorted(n for n in os.listdir(tiles) if n.startswith("mine_"))
    assert len(names) == 2
    assert "mine_esri_20250101_z10-10_wgs84.mbtiles" not in names   # 最老的先删
    assert "mine_esri_20250103_z10-10_wgs84.mbtiles" in names


def test_max_tiles_guard(tmp_path):
    cfg = _cfg(tmp_path, bbox=(-179.0, -80.0, 179.0, 80.0), zoom_min=12, zoom_max=14)
    res = TileManager(cfg, fetch=FakeFetch()).sync()
    assert res["ok"] is False and "max_tiles" in res["reason"]


def test_no_bbox_configured(tmp_path):
    cfg = _cfg(tmp_path, bbox=None)
    res = TileManager(cfg, fetch=FakeFetch()).sync()
    assert res["ok"] is False and "bbox" in res["reason"]
