import sqlite3

import pytest

from gnss_hmi.tiles import TileStore


def _mk_mbtiles(path):
    db = sqlite3.connect(path)
    db.execute("CREATE TABLE tiles (zoom_level INT, tile_column INT, tile_row INT, tile_data BLOB)")
    db.execute("INSERT INTO tiles VALUES (2, 1, 2, ?)", (b"PNGDATA",))  # TMS row 2 == XYZ y 1
    db.commit(); db.close()


def test_get_flips_tms(tmp_path):
    p = tmp_path / "m.mbtiles"; _mk_mbtiles(p)
    t = TileStore(str(p))
    assert t.get(2, 1, 1) == b"PNGDATA"       # y=1 -> row = 4-1-1 = 2
    assert t.get(2, 0, 0) is None
    t.close()


def test_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        TileStore(str(tmp_path / "none.mbtiles"))
    with pytest.raises(FileNotFoundError):
        TileStore("")
