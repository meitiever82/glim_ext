"""在线 XYZ 底图预热：按 bbox/zoom 拉瓦片写入本地 MBTiles 缓存。

设计要点（plan Task 8 / spec §5）：
- 只在显式调用（CLI 或 tiles.warm_on_start）时联网；服务运行时只读本地文件。
- 新版本先写临时文件、算 SHA-256，再 ``os.replace`` 成带版本名的文件，
  最后原子切换 ``current.mbtiles`` 软链 —— 任何一步失败都不会破坏可用旧缓存。
- 保留最近 2 个版本；``index.json`` 记录版本、SHA、瓦片数与归属。
- 断网/源不可用：已有缓存继续可用；完全无缓存则返回失败，页面退网格。
"""
from __future__ import annotations

import hashlib
import json
import logging
import math
import os
import shutil
import sqlite3
import time
import urllib.request
from datetime import datetime, timezone

_logger = logging.getLogger(__name__)

CURRENT_NAME = "current.mbtiles"
INDEX_NAME = "index.json"
KEEP_VERSIONS = 2
MAX_LAT = 85.05112878


def _http_get(url: str, timeout: float) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": "gnss-hmi/0.2"})
    with urllib.request.urlopen(req, timeout=timeout) as resp:   # noqa: S310
        return resp.read()


def _sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _xtile(lon: float, z: int) -> float:
    return (lon + 180.0) / 360.0 * (2 ** z)


def _ytile(lat: float, z: int) -> float:
    lat = max(min(lat, MAX_LAT), -MAX_LAT)
    r = math.radians(lat)
    return (1.0 - math.log(math.tan(r) + 1.0 / math.cos(r)) / math.pi) / 2.0 * (2 ** z)


def tile_range(bbox: tuple[float, float, float, float],
               z: int) -> tuple[int, int, int, int]:
    """bbox=(min_lon,min_lat,max_lon,max_lat) → (x0, x1, y0, y1)，含端点。"""
    min_lon, min_lat, max_lon, max_lat = bbox
    n = 2 ** z - 1

    def clamp(v: float) -> int:
        return max(0, min(n, int(v)))

    return (clamp(_xtile(min_lon, z)), clamp(_xtile(max_lon, z)),
            clamp(_ytile(max_lat, z)), clamp(_ytile(min_lat, z)))


def planned_keys(bbox, zooms) -> list[tuple[int, int, int]]:
    keys = []
    for z in range(int(zooms[0]), int(zooms[1]) + 1):
        x0, x1, y0, y1 = tile_range(tuple(bbox), z)
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                keys.append((z, x, y))
    return keys


def _connect_rw(path: str) -> sqlite3.Connection:
    db = sqlite3.connect(path)
    db.execute("CREATE TABLE IF NOT EXISTS metadata (name TEXT, value TEXT)")
    db.execute("CREATE TABLE IF NOT EXISTS tiles (zoom_level INTEGER, "
               "tile_column INTEGER, tile_row INTEGER, tile_data BLOB)")
    db.execute("CREATE UNIQUE INDEX IF NOT EXISTS tiles_idx ON tiles "
               "(zoom_level, tile_column, tile_row)")
    return db


def _existing_keys(db: sqlite3.Connection) -> set[tuple[int, int, int]]:
    rows = db.execute("SELECT zoom_level, tile_column, tile_row FROM tiles").fetchall()
    # 库内是 TMS 行号，换回 XYZ y 便于与计划集合比较
    return {(int(z), int(x), (2 ** int(z)) - 1 - int(row)) for z, x, row in rows}


def describe_store(path: str, cfg=None) -> dict:
    """给 /api/tiles_info 用：文件存在即 available，附版本信息。"""
    if not path or not os.path.isfile(path):
        return {"available": False}
    st = os.stat(path)
    info = {
        "available": True,
        "file": os.path.basename(path),
        "bytes": int(st.st_size),
        "updated": int(st.st_mtime),
    }
    if cfg is not None:
        info["attribution"] = cfg.attribution
        info["source"] = cfg.source_name
    idx = _read_index(os.path.dirname(path) or ".")
    if idx and idx.get("current") == os.path.basename(path):
        for k in ("sha256", "tiles", "zooms", "bbox", "attribution", "updated"):
            if k in idx:
                info[k] = idx[k]
    return info


def _read_index(cache_dir: str) -> dict:
    try:
        with open(os.path.join(cache_dir, INDEX_NAME), encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


class TileManager:
    def __init__(self, cfg, fetch=None, now=None) -> None:
        self.cfg = cfg
        self._fetch = fetch or _http_get
        self._now = now or time.time

    # ---------- 查询 ----------

    def current_path(self) -> str | None:
        p = os.path.join(self.cfg.cache_dir, CURRENT_NAME)
        return p if os.path.exists(p) else None

    def info(self) -> dict:
        info = describe_store(self.current_path() or "", self.cfg)
        if info.get("available") and not info.get("sha256"):
            try:
                info["tiles"] = _count_tiles(self.current_path())
            except sqlite3.DatabaseError:
                return {"available": False, "reason": "corrupt tiles file"}
        return info

    def _url(self, z: int, x: int, y: int) -> str:
        return (self.cfg.source_url.replace("{z}", str(z))
                .replace("{x}", str(x)).replace("{y}", str(y)))

    # ---------- 预热 ----------

    def sync(self, bbox=None, zooms=None, fill_missing: bool = True) -> dict:
        cfg = self.cfg
        if not getattr(cfg, "enabled", True):
            return {"ok": False, "reason": "tiles disabled"}
        bbox = tuple(bbox or cfg.bbox or ())
        zooms = zooms or (cfg.zoom_min, cfg.zoom_max)
        if len(bbox) != 4:
            return {"ok": False, "reason": "tiles.bbox 未配置（min_lon,min_lat,max_lon,max_lat）"}
        keys = planned_keys(bbox, zooms)
        if len(keys) > cfg.max_tiles:
            return {"ok": False,
                    "reason": f"需要 {len(keys)} 块瓦片 > max_tiles={cfg.max_tiles}，"
                              f"请缩小 bbox 或 zoom 范围"}

        os.makedirs(cfg.cache_dir, exist_ok=True)
        cur = self.current_path()
        existing: set[tuple[int, int, int]] = set()
        if cur and fill_missing:
            existing = self._read_keys(cur)
        todo = [k for k in keys if k not in existing] if fill_missing else keys
        if not todo and cur:
            return {"ok": True, "unchanged": True, "tiles": len(existing),
                    "path": cur, "reason": "缓存已完整"}

        tmp = os.path.join(cfg.cache_dir, f".build-{os.getpid()}-{int(self._now())}.mbtiles")
        try:
            if cur and existing:
                shutil.copyfile(cur, tmp)
                try:
                    db = _connect_rw(tmp)
                except sqlite3.DatabaseError:
                    os.unlink(tmp)
                    db = _connect_rw(tmp)
            else:
                if os.path.exists(tmp):
                    os.unlink(tmp)
                db = _connect_rw(tmp)

            fetched = failed = 0
            try:
                for z, x, y in todo:
                    try:
                        data = self._fetch(self._url(z, x, y), cfg.timeout_s)
                    except Exception as exc:                      # 单块失败不中断
                        failed += 1
                        _logger.debug("tile %s/%s/%s failed: %s", z, x, y, exc)
                        continue
                    if not data:
                        failed += 1
                        continue
                    db.execute("INSERT OR REPLACE INTO tiles VALUES (?,?,?,?)",
                               (z, x, (2 ** z) - 1 - y, sqlite3.Binary(data)))
                    fetched += 1
                    if fetched % 100 == 0:
                        db.commit()
                if fetched == 0 and failed and not existing:
                    db.close()
                    os.unlink(tmp)
                    return {"ok": False, "reason": f"{failed} 块瓦片全部拉取失败（无网/源不可用）",
                            "failed": failed}
                self._write_metadata(db, bbox, zooms)
                db.commit()
                total = int(db.execute("SELECT COUNT(*) FROM tiles").fetchone()[0])
            finally:
                db.close()

            sha = _sha256(tmp)
            name = (f"mine_{cfg.source_name}_"
                    f"{datetime.now(timezone.utc).strftime('%Y%m%d')}_"
                    f"z{int(zooms[0])}-{int(zooms[1])}_wgs84.mbtiles")
            final = os.path.join(cfg.cache_dir, name)
            os.replace(tmp, final)
        except Exception:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise

        self._point_current(name)
        self._write_index(name, sha, total, bbox, zooms)
        self._prune(keep=KEEP_VERSIONS)
        return {"ok": True, "path": final, "fetched": fetched, "failed": failed,
                "tiles": total, "sha256": sha, "unchanged": False}

    def _read_keys(self, path: str) -> set[tuple[int, int, int]]:
        try:
            db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
            try:
                return _existing_keys(db)
            finally:
                db.close()
        except sqlite3.DatabaseError:
            return set()          # 坏文件：视为空缓存，重建

    def _write_metadata(self, db, bbox, zooms) -> None:
        meta = {
            "name": f"gnss_hmi {self.cfg.source_name} basemap",
            "format": getattr(self.cfg, "format", "jpg"),
            "type": "baselayer",
            "version": "1.0",
            "attribution": self.cfg.attribution,
            "minzoom": str(int(zooms[0])),
            "maxzoom": str(int(zooms[1])),
            "bounds": ",".join(f"{v:.6f}" for v in bbox),
        }
        for k, v in meta.items():
            db.execute("DELETE FROM metadata WHERE name=?", (k,))
            db.execute("INSERT INTO metadata VALUES (?,?)", (k, v))

    def _point_current(self, name: str) -> None:
        link = os.path.join(self.cfg.cache_dir, CURRENT_NAME)
        tmp = link + ".tmp"
        if os.path.lexists(tmp):
            os.unlink(tmp)
        os.symlink(name, tmp)                 # 相对软链：cache_dir 可整体搬移
        os.replace(tmp, link)                 # 原子切换

    def _write_index(self, name: str, sha: str, tiles: int, bbox, zooms) -> None:
        idx = {
            "current": name,
            "sha256": sha,
            "tiles": tiles,
            "zooms": [int(zooms[0]), int(zooms[1])],
            "bbox": [float(v) for v in bbox],
            "source": self.cfg.source_name,
            "source_url": self.cfg.source_url,
            "attribution": self.cfg.attribution,
            "updated": int(self._now()),
        }
        path = os.path.join(self.cfg.cache_dir, INDEX_NAME)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(idx, f, ensure_ascii=False, indent=2)
        os.replace(tmp, path)

    def _prune(self, keep: int = KEEP_VERSIONS) -> list[str]:
        files = [os.path.join(self.cfg.cache_dir, n)
                 for n in os.listdir(self.cfg.cache_dir)
                 if n.startswith("mine_") and n.endswith(".mbtiles")]
        files.sort(key=lambda p: os.stat(p).st_mtime, reverse=True)
        removed = []
        for p in files[keep:]:
            try:
                os.unlink(p)
                removed.append(p)
            except OSError:
                pass
        return removed


def _count_tiles(path: str) -> int:
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    try:
        return int(db.execute("SELECT COUNT(*) FROM tiles").fetchone()[0])
    finally:
        db.close()


def main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    from gnss_hmi.config import load_config

    ap = argparse.ArgumentParser(description="预热在线 XYZ 底图到本地 MBTiles 缓存")
    ap.add_argument("--config", default="config/gnss_hmi.yaml")
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    res = TileManager(cfg.tiles).sync()
    print(json.dumps(res, ensure_ascii=False, indent=2))
    return 0 if res.get("ok") else 1


if __name__ == "__main__":
    import sys
    sys.exit(main(sys.argv[1:]))
