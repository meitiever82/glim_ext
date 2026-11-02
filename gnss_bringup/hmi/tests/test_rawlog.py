import json

from gnss_hmi.storage.rawlog import RawLogWriter


class FakeClock:
    def __init__(self, t: float):
        self.t = t
    def __call__(self) -> float:
        return self.t


def test_append_writes_raw_and_index(tmp_path):
    clock = FakeClock(1756699200.0)  # 2026-09-01 12:00:00 +0800 (local test box TZ-agnostic: just a fixed t)
    w = RawLogWriter(tmp_path, "corr", ext="rtcm3", clock=clock)
    w.append(b"\xd3\x00\x01", msg_type=1074)
    w.append(b"\xab\xcd", msg_type=1005)
    w.close()
    days = list(tmp_path.iterdir())
    assert len(days) == 1
    binfile = next(days[0].glob("corr_*.rtcm3"))
    assert binfile.read_bytes() == b"\xd3\x00\x01\xab\xcd"
    idx = [json.loads(l) for l in
           next(days[0].glob("corr_*.idx.jsonl")).read_text().splitlines()]
    assert idx[0] == {"t": 1756699200.0, "type": 1074, "off": 0, "len": 3}
    assert idx[1]["off"] == 3 and idx[1]["len"] == 2


def test_index_false_skips_sidecar_file(tmp_path):
    clock = FakeClock(1756699200.0)
    w = RawLogWriter(tmp_path, "can0", ext="log", clock=clock, index=False)
    w.append(b"line-one\n")
    w.append(b"line-two\n")
    w.close()
    day = next(tmp_path.iterdir())
    assert list(day.glob("*.idx.jsonl")) == []
    assert next(day.glob("can0_*.log")).read_bytes() == b"line-one\nline-two\n"


def test_flush_every_batches_flushes(tmp_path):
    clock = FakeClock(1756699200.0)
    w = RawLogWriter(tmp_path, "corr", clock=clock, flush_every=3)
    w.append(b"a")  # triggers rotation + first append; pending=1, no flush yet
    flush_calls: list[int] = []
    orig_flush = w._file.flush
    w._file.flush = lambda: (flush_calls.append(1), orig_flush())[-1]
    w.append(b"b")  # pending=2, still no flush
    assert flush_calls == []
    w.append(b"c")  # pending=3 -> flush
    assert flush_calls == [1]
    w.append(b"d")  # pending=1 again after reset, no flush
    assert flush_calls == [1]
    w.close()


def test_hour_rotation(tmp_path):
    clock = FakeClock(1756699200.0)
    w = RawLogWriter(tmp_path, "corr", clock=clock)
    w.append(b"a")
    clock.t += 3600
    w.append(b"b")
    w.close()
    bins = sorted(p.name for d in tmp_path.iterdir() for p in d.glob("corr_*.bin"))
    assert len(bins) == 2 and bins[0] != bins[1]
