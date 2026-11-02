"""GeoTIFF -> MBTiles import (Plan T12).

Coordinate safety: the source SRS must be explicit and WGS-84 (EPSG:4326) or
Web Mercator (EPSG:3857). GCJ-02 / BD-09 have no authoritative EPSG code, so
requiring an explicit allow-listed EPSG also rejects them by construction.

GDAL is optional; the validation helpers stay importable (and testable) without
it. The conversion only runs when `gdal` is installed.
"""
from __future__ import annotations

import math
from pathlib import Path

from gnss_hmi.tiles.manager import install_cache

ALLOWED_EPSG = {4326, 3857}


def validate_epsg(epsg: int | str | None) -> int:
    try:
        code = int(epsg)
    except (TypeError, ValueError):
        raise ValueError("EPSG code is required (explicit 4326 or 3857)") from None
    if code not in ALLOWED_EPSG:
        raise ValueError(
            f"EPSG:{code} not allowed; use 4326 (WGS-84) or 3857 (Web Mercator)")
    return code


def validate_bbox(bbox: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    try:
        west, south, east, north = (float(v) for v in bbox)
    except (TypeError, ValueError):
        raise ValueError("bbox must be west,south,east,north") from None
    if not all(math.isfinite(v) for v in (west, south, east, north)):
        raise ValueError("bbox contains NaN/Inf")
    if not (-180 <= west < east <= 180 and -90 <= south < north <= 90):
        raise ValueError("bbox out of range or not increasing")
    return west, south, east, north


def import_geotiff(src: str, root: str, *, epsg: int | str | None = None,
                   zoom: int = 17, name: str = "geotiff") -> str:
    """Warp the source to EPSG:3857, translate to MBTiles, publish atomically.

    Returns the installed version path. Raises RuntimeError when GDAL is missing
    or the source has no valid SRS.
    """
    code = validate_epsg(epsg)
    try:
        from osgeo import gdal, osr  # type: ignore
    except ImportError as e:
        raise RuntimeError("GDAL (python3-gdal) is required for GeoTIFF import") from e
    ds = gdal.Open(src)
    if ds is None:
        raise RuntimeError(f"cannot open {src}")
    gt = ds.GetGeoTransform()
    if not all(math.isfinite(v) for v in gt):
        raise RuntimeError("GeoTIFF geotransform is invalid")
    if code == 4326:
        validate_bbox((gt[0], gt[3] + ds.RasterYSize * gt[5],
                       gt[0] + ds.RasterXSize * gt[1], gt[3]))
    if not ds.GetProjection():
        raise RuntimeError("GeoTIFF has no CRS; georeference it before import")
    # The source CRS must match the caller's explicit declaration.
    sr = osr.SpatialReference(wkt=ds.GetProjection())
    sr.AutoIdentifyEPSG()
    if sr.GetAuthorityCode(None) not in (None, str(code)):
        raise RuntimeError(
            f"source CRS is EPSG:{sr.GetAuthorityCode(None)}, expected EPSG:{code}")
    ds = None

    root_p = Path(root)
    root_p.mkdir(parents=True, exist_ok=True)
    warped = root_p / ".geotiff.warp.tif"
    out = root_p / ".geotiff.tmp.mbtiles"
    for p in (warped, out):
        p.unlink(missing_ok=True)
    tmp = gdal.Warp(str(warped), src, dstSRS="EPSG:3857")
    if tmp is None:
        raise RuntimeError("gdal.Warp failed")
    tmp = None
    drv = gdal.GetDriverByName("MBTILES")
    if drv is None:
        raise RuntimeError("GDAL MBTILES driver unavailable")
    out_ds = drv.CreateCopy(str(out), gdal.Open(str(warped)))
    out_ds = None
    warped.unlink(missing_ok=True)
    version = install_cache(str(root_p), str(out), name=name)
    out.unlink(missing_ok=True)
    return version


def _main() -> int:
    import argparse
    ap = argparse.ArgumentParser(description="Import a mine GeoTIFF into the HMI tile cache")
    ap.add_argument("src", help="source GeoTIFF")
    ap.add_argument("--root", required=True, help="tile root (directory)")
    ap.add_argument("--epsg", default=None,
                    help="explicit EPSG code: 4326 or 3857 (required)")
    ap.add_argument("--name", default="geotiff")
    args = ap.parse_args()
    try:
        print(import_geotiff(args.src, args.root, epsg=args.epsg, name=args.name))
    except (ValueError, RuntimeError) as e:
        print(f"import failed: {e}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
