import sqlite3

import pytest

from gnss_hmi.tiles.geotiff import validate_bbox, validate_epsg
from gnss_hmi.tiles.manager import install_cache, sha_ok


def test_epsg_allowlist_rejects_gcj02_and_unknown():
    assert validate_epsg(4326) == 4326
    assert validate_epsg("3857") == 3857
    for bad in (None, "", "GCJ-02", 4490, 0):
        with pytest.raises(ValueError):
            validate_epsg(bad)


def test_bbox_validation():
    assert validate_bbox((-1, -1, 1, 1)) == (-1.0, -1.0, 1.0, 1.0)
    with pytest.raises(ValueError):
        validate_bbox((1, 1, -1, -1))       # not increasing
    with pytest.raises(ValueError):
        validate_bbox((-181, 0, 1, 1))      # out of range
    with pytest.raises(ValueError):
        validate_bbox((0, 0, float("nan"), 1))


def test_install_cache_publishes_and_prunes(tmp_path):
    src = tmp_path / "import.mbtiles"
    db = sqlite3.connect(src)
    db.execute("CREATE TABLE tiles (zoom_level INT, tile_column INT, tile_row INT, tile_data BLOB)")
    db.execute("INSERT INTO tiles VALUES (2, 1, 2, ?)", (b"X",))
    db.commit(); db.close()
    v = install_cache(str(tmp_path / "root"), str(src), name="geo")
    assert v.endswith(".mbtiles")
    cur = tmp_path / "root" / "current.mbtiles"
    assert sha_ok(cur)
    # second import prunes to two versions
    install_cache(str(tmp_path / "root"), str(src), name="geo2")
    versions = list((tmp_path / "root" / "versions").glob("*.mbtiles"))
    assert len(versions) == 2
