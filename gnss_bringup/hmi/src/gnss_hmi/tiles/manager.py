"""Offline basemap cache: bbox/zoom XYZ download -> MBTiles -> atomic switch.

The HMI only reads the active `current.mbtiles`; refreshing a mine-area cache is
an explicit build step. Every build is written to a temp file, verified against
a SHA-256 sidecar, then atomically moved into place, so a failed or partial
download can never replace a working cache.
"""
from __future__ import annotations

import hashlib
import math
import os
import shutil
import sqlite3
import time
import urllib.error
import urllib.request
from pathlib import Path

DEFAULT_SOURCE = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
                  "World_Imagery/MapServer/tile/{z}/{y}/{x}")
KEEP_VERSIONS = 2


def lonlat_to_tile(lon: float, lat: float, z: int) -> tuple[int, int]:
    """EPSG:3857 slippy-map XYZ tile for a WGS-84 lon/lat."""
    n = 2 ** z
    lat = max(min(lat, 85.05112878), -85.05112878)
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n)
    return min(max(x, 0), n - 1), min(max(y, 0), n - 1)


def tile_range(bbox: tuple[float, float, float, float], z: int) -> tuple[range, range]:
    """Inclusive x/y tile ranges for a (west, south, east, north) bbox."""
    west, south, east, north = bbox
    x0, y1 = lonlat_to_tile(west, south, z)
    x1, y0 = lonlat_to_tile(east, north, z)
    return range(min(x0, x1), max(x0, x1) + 1), range(min(y0, y1), max(y0, y1) + 1)


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _sidecar(path: Path) -> Path:
    return path.with_name(path.name + ".sha256")


def _write_sidecar(path: Path, digest: str) -> None:
    _sidecar(path).write_text(f"{digest}  {path.name}\n")


def sha_ok(path: Path) -> bool:
    """A user-supplied MBTiles without a sidecar is trusted; one with a
    sidecar must match it (a truncated copy is rejected)."""
    p = Path(path)
    if not p.is_file():
        return False
    side = _sidecar(p)
    if not side.is_file():
        return True
    text = side.read_text().strip()
    want = text.split()[0] if text else ""
    return bool(want) and _sha256(p) == want


def _versions_dir(root: Path) -> Path:
    return root / "versions"


def resolve_tiles_path(configured: str) -> str | None:
    """Active cache to serve: the configured file if it verifies, else the
    newest intact version next to it, else None (page falls back to grid)."""
    p = Path(configured) if configured else None
    if p is None:
        return None
    if sha_ok(p):
        return str(p)
    vdir = _versions_dir(p.parent)
    if vdir.is_dir():
        for v in sorted(vdir.glob("*.mbtiles"),
                        key=lambda x: x.stat().st_mtime, reverse=True):
            if sha_ok(v):
                return str(v)
    return None


def _http_get(url: str, timeout: float) -> bytes | None:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            data = resp.read()
            return data or None
    except urllib.error.HTTPError as e:
        if e.code == 404:
            return None   # outside source coverage: leave a gap
        raise


def refresh_due(state_path: str | os.PathLike, period_s: float,
                now: float | None = None) -> bool:
    """True when the last successful maintain is older than period_s (or never
    ran). The timestamp is written only after a build succeeds."""
    try:
        last = float(Path(state_path).read_text().strip())
    except (OSError, ValueError):
        return True
    return (now if now is not None else time.time()) - last >= period_s


def maintain(root: str, bbox: tuple[float, float, float, float], zooms: list[int],
             state_path: str | os.PathLike, *, period_s: float = 7 * 24 * 3600,
             now: float | None = None, source: str = DEFAULT_SOURCE,
             http_get=None) -> str | None:
    """Weekly-style cache refresh: skip when not due, otherwise build and
    stamp the state file on success so a failed run retries next check."""
    if not refresh_due(state_path, period_s, now):
        return None
    out = TileManager(root, source, http_get=http_get).build(bbox, zooms)
    if out:
        Path(state_path).write_text(f"{now if now is not None else time.time():.0f}\n")
    return out


def install_cache(root: str, source_file: str | os.PathLike,
                  name: str = "mine") -> str:
    """Publish an already-built MBTiles (e.g. a GeoTIFF import) as the active
    cache: hash -> version -> atomic current.mbtiles, keeping two versions."""
    r = Path(root)
    _versions_dir(r).mkdir(parents=True, exist_ok=True)
    db = sqlite3.connect(source_file)
    try:
        db.execute("SELECT COUNT(*) FROM tiles").fetchone()   # reject non-MBTiles
    finally:
        db.close()
    digest = _sha256(Path(source_file))
    stamp = int(time.time())
    version = _versions_dir(r) / f"{name}-{stamp}.mbtiles"
    shutil.copy2(source_file, version)
    _write_sidecar(version, digest)
    cur_tmp = r / ".current.tmp.mbtiles"
    shutil.copy2(version, cur_tmp)
    os.replace(cur_tmp, r / "current.mbtiles")
    _write_sidecar(r / "current.mbtiles", digest)
    versions = sorted(_versions_dir(r).glob("*.mbtiles"),
                      key=lambda p: p.stat().st_mtime, reverse=True)
    for old in versions[KEEP_VERSIONS:]:
        old.unlink(missing_ok=True)
        _sidecar(old).unlink(missing_ok=True)
    return str(version)


class TileManager:
    def __init__(self, root: str, source: str = DEFAULT_SOURCE, *,
                 timeout: float = 10.0, http_get=None) -> None:
        self.root = Path(root)
        self.source = source
        self.timeout = timeout
        self._get = http_get or (lambda url: _http_get(url, timeout))
        self.root.mkdir(parents=True, exist_ok=True)
        _versions_dir(self.root).mkdir(exist_ok=True)

    @property
    def current(self) -> Path:
        return self.root / "current.mbtiles"

    def active(self) -> str | None:
        return resolve_tiles_path(str(self.current))

    def _init_db(self, db: sqlite3.Connection) -> None:
        db.execute("CREATE TABLE IF NOT EXISTS tiles (zoom_level INT, "
                   "tile_column INT, tile_row INT, tile_data BLOB)")
        db.execute("CREATE UNIQUE INDEX IF NOT EXISTS idx_tiles "
                   "ON tiles (zoom_level, tile_column, tile_row)")
        db.execute("CREATE TABLE IF NOT EXISTS metadata (name TEXT, value TEXT)")

    def _carry_over(self, db: sqlite3.Connection) -> int:
        """Seed the new build with the currently served tiles so a build only
        refreshes/patches the requested bbox instead of wiping cached areas."""
        path = self.active()
        if not path:
            return 0
        db.execute("ATTACH DATABASE ? AS cur", (path,))
        try:
            db.execute("INSERT OR IGNORE INTO tiles "
                       "SELECT zoom_level, tile_column, tile_row, tile_data FROM cur.tiles")
            n = db.execute("SELECT COUNT(*) FROM tiles").fetchone()[0]
            db.commit()   # DETACH needs the read transaction closed first
            return n
        finally:
            db.execute("DETACH DATABASE cur")

    def build(self, bbox: tuple[float, float, float, float],
              zooms: list[int], name: str = "mine") -> str | None:
        """Fetch the bbox at each zoom and atomically install a new cache.

        Returns the version path on success, or the previously active path when
        the source is unreachable but a verified cache exists. Returns None when
        there is nothing usable (caller renders the coordinate grid).
        """
        tmp = self.root / ".build.tmp.mbtiles"
        tmp.unlink(missing_ok=True)
        fetched = failed = 0
        try:
            db = sqlite3.connect(tmp)
            try:
                self._init_db(db)
                self._carry_over(db)
                for z in sorted(set(int(v) for v in zooms)):
                    xr, yr = tile_range(bbox, z)
                    for x in xr:
                        for y in yr:
                            url = self.source.format(z=z, x=x, y=y)
                            try:
                                data = self._get(url)
                            except Exception:
                                failed += 1
                                continue
                            if not data:
                                continue
                            db.execute(
                                "INSERT OR REPLACE INTO tiles VALUES (?,?,?,?)",
                                (z, x, (2 ** z) - 1 - y, data))
                            fetched += 1
                rows = db.execute("SELECT COUNT(*) FROM tiles").fetchone()[0]
                for k, v in (("name", name), ("format", "png"),
                             ("minzoom", str(min(zooms))), ("maxzoom", str(max(zooms))),
                             ("bbox", ",".join(f"{v:.6f}" for v in bbox)),
                             ("created", f"{time.time():.0f}"),
                             ("tiles", str(rows))):
                    db.execute("INSERT OR REPLACE INTO metadata VALUES (?,?)", (k, v))
                db.commit()
            finally:
                db.close()
            if rows == 0:
                tmp.unlink(missing_ok=True)
                return self.active()   # offline with no cache -> grid fallback
            digest = _sha256(tmp)
            stamp = int(time.time())
            version = _versions_dir(self.root) / f"{name}-{stamp}.mbtiles"
            os.replace(tmp, version)
            _write_sidecar(version, digest)
            # Atomic publish: copy to a temp name in the same dir, then rename.
            cur_tmp = self.root / ".current.tmp.mbtiles"
            shutil.copy2(version, cur_tmp)
            os.replace(cur_tmp, self.current)
            _write_sidecar(self.current, digest)
            self._prune()
            return str(version)
        except Exception:
            tmp.unlink(missing_ok=True)
            return self.active()

    def _prune(self) -> None:
        versions = sorted(_versions_dir(self.root).glob("*.mbtiles"),
                          key=lambda p: p.stat().st_mtime, reverse=True)
        for old in versions[KEEP_VERSIONS:]:
            old.unlink(missing_ok=True)
            _sidecar(old).unlink(missing_ok=True)


def _main() -> int:
    """python -m gnss_hmi.tiles.manager --root DIR --bbox W,S,E,N --zooms 14-18"""
    import argparse
    ap = argparse.ArgumentParser(description="Build/refresh the offline MBTiles cache")
    ap.add_argument("--root", default=os.environ.get("GNSS_HMI_TILES_ROOT", ""),
                    required=not os.environ.get("GNSS_HMI_TILES_ROOT"))
    ap.add_argument("--bbox", default=os.environ.get("GNSS_HMI_TILES_BBOX"),
                    help="west,south,east,north")
    ap.add_argument("--zooms", default=os.environ.get("GNSS_HMI_TILES_ZOOMS", "14-18"),
                    help="e.g. 14-18 or 14,16,18")
    ap.add_argument("--source", default=DEFAULT_SOURCE)
    args = ap.parse_args()
    if not args.bbox:
        ap.error("--bbox (or GNSS_HMI_TILES_BBOX) is required")
    bbox = tuple(float(v) for v in args.bbox.split(","))
    zs = ([int(v) for v in args.zooms.split("-")]
          if "-" in args.zooms else [int(v) for v in args.zooms.split(",")])
    if len(zs) == 2 and "-" in args.zooms:
        zs = list(range(zs[0], zs[1] + 1))
    out = TileManager(args.root, args.source).build(bbox, zs)
    print(out or "no cache (source unavailable and no previous version)")
    return 0 if out else 1


if __name__ == "__main__":
    raise SystemExit(_main())
