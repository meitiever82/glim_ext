import datetime
from collections import namedtuple

from gnss_hmi.storage.cleanup import cleanup_logs

Usage = namedtuple("Usage", "total used free")
TODAY = datetime.date(2026, 9, 10)


def _mk_days(tmp_path, *days):
    for d in days:
        (tmp_path / d).mkdir()
        (tmp_path / d / "x.bin").write_bytes(b"1")


def test_deletes_beyond_retention(tmp_path):
    _mk_days(tmp_path, "20260820", "20260901", "20260910")
    deleted = cleanup_logs(tmp_path, retention_days=14, watermark_pct=85.0,
                           disk_usage=lambda p: Usage(100, 10, 90), today=TODAY)
    assert [p.name for p in deleted] == ["20260820"]
    assert sorted(p.name for p in tmp_path.iterdir()) == ["20260901", "20260910"]


def test_deletes_oldest_when_over_watermark(tmp_path):
    _mk_days(tmp_path, "20260908", "20260909", "20260910")
    calls = [Usage(100, 90, 10), Usage(100, 80, 20)]  # over, then under after one delete
    deleted = cleanup_logs(tmp_path, retention_days=14, watermark_pct=85.0,
                           disk_usage=lambda p: calls.pop(0), today=TODAY)
    assert [p.name for p in deleted] == ["20260908"]


def test_never_deletes_today(tmp_path):
    _mk_days(tmp_path, "20260910")
    deleted = cleanup_logs(tmp_path, retention_days=0, watermark_pct=0.0,
                           disk_usage=lambda p: Usage(100, 99, 1), today=TODAY)
    assert deleted == []
