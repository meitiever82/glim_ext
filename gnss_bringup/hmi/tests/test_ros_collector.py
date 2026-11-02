"""Unit tests for the read-only ROS collector (T2/T3/T4).

Uses plain stand-ins for ROS messages so the suite needs no rclpy runtime.
"""
import math
import types

from gnss_hmi.collectors.ros_collector import (
    RosCollector, TopicCfg, _LineBuffer, diag_state, rtkfix_state)


class FakeClock:
    def __init__(self, t: float) -> None:
        self._t = t

    def now(self):
        return types.SimpleNamespace(nanoseconds=int(self._t * 1e9))

    def advance(self, dt: float) -> None:
        self._t += dt


class FakeNode:
    def __init__(self, t: float = 100.0) -> None:
        self.clock = FakeClock(t)

    def get_clock(self):
        return self.clock


def _msg(**kw):
    base = dict(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=100, nanosec=0)),
        gnss_time=0.0, quality=4, raw_status=4,
        latitude=44.5, longitude=90.2, altitude=1200.0,
        sigma_enu=[0.01, 0.02, 0.03], diff_age=1.0,
        sats_used=20, sats_main=15, sats_aux=5,
        heading=90.0, heading_sigma=0.1, heading_valid=True,
        ratio=0.0,
    )
    base.update(kw)
    return types.SimpleNamespace(**base)


def _collector() -> RosCollector:
    return RosCollector("test", (), lambda e: None, lambda *a: None)


# --- T2: RtkFix mapping ----------------------------------------------------
def test_quality_all_five_levels_pass_through():
    c = _collector()
    node = FakeNode()
    for q in range(5):
        c._on_rtkfix("app", _msg(quality=q), node, "/app")
    snap = c.snapshot()
    assert snap["sources"]["app"]["q"] == 4


def test_sigma_is_e_n_u_order_and_named():
    c = _collector()
    c._on_rtkfix("app", _msg(sigma_enu=[0.01, 0.02, 0.03]), FakeNode(), "/app")
    st = c.snapshot()["sources"]["app"]
    assert (st["sigma_e"], st["sigma_n"], st["sigma_u"]) == (0.01, 0.02, 0.03)


def test_ratio_and_gnss_time_zero_become_none():
    c = _collector()
    c._on_rtkfix("fused", _msg(ratio=0.0, gnss_time=0.0), FakeNode(), "/fused")
    st = c.snapshot()["sources"]["fused"]
    assert st["ratio"] is None and st["gnss_time"] is None


def test_ratio_positive_is_kept():
    c = _collector()
    c._on_rtkfix("app", _msg(ratio=3.5), FakeNode(), "/app")
    assert c.snapshot()["sources"]["app"]["ratio"] == 3.5


def test_nan_position_rejected():
    assert rtkfix_state(_msg(latitude=float("nan")), 1.0) is None
    assert rtkfix_state(_msg(longitude=float("inf")), 1.0) is None


def test_nan_sigma_becomes_none():
    st = rtkfix_state(_msg(sigma_enu=[float("nan"), 0.02, float("inf")]), 1.0)
    assert st["sigma_e"] is None and st["sigma_u"] is None


def test_invalid_heading_hidden():
    st = rtkfix_state(_msg(heading_valid=False), 1.0)
    assert st["heading"] is None and st["heading_valid"] is False


def test_two_sources_do_not_overwrite_each_other():
    c = _collector()
    node = FakeNode()
    c._on_rtkfix("app", _msg(quality=3, sats_used=9), node, "/app")
    node.clock.advance(0.5)
    c._on_rtkfix("fused", _msg(quality=4, sats_used=21), node, "/fused")
    snap = c.snapshot()
    assert snap["sources"]["app"]["q"] == 3
    assert snap["sources"]["app"]["sats"] == 9
    assert snap["sources"]["fused"]["q"] == 4
    assert snap["sources"]["fused"]["sats"] == 21


# --- T3: $SAT Reassembly ---------------------------------------------------
def test_line_buffer_reassembles_split_lines():
    buf = _LineBuffer()
    assert buf.feed(b"$SAT,1,2,$G01") == []
    assert buf.feed(b",10,20\n$SAT,1,2,$G02,10,20\n$SAT,1,2,$G03") == [
        "$SAT,1,2,$G01,10,20", "$SAT,1,2,$G02,10,20"]
    assert buf.feed(b",10,20\n") == ["$SAT,1,2,$G03,10,20"]


def test_line_buffer_handles_empty_chunks():
    buf = _LineBuffer()
    assert buf.feed(b"") == []


def _sat_line(tow, sat_num, snr=40.0, valid=1):
    return (f"$SAT,{tow},{tow},G{sat_num:02d},1,120,45,0.1,{valid},1,{snr},"
            f"0,0,0,0,0,0\n")


def test_stat_stream_groups_by_tow_and_keeps_one_point_per_sat():
    c = _collector()
    node = FakeNode()
    msg = types.SimpleNamespace(data=(_sat_line(10, 1) + _sat_line(10, 1)).encode())
    c._on_stat(msg, node, "/stat")
    sats = c.snapshot()["stat"]["sats"]
    assert len(sats) == 1
    msg2 = types.SimpleNamespace(data=(_sat_line(10, 2)).encode())
    c._on_stat(msg2, node, "/stat")
    assert len(c.snapshot()["stat"]["sats"]) == 2
    # new epoch (tow changes) resets the accumulation
    msg3 = types.SimpleNamespace(data=(
        _sat_line(11, 1) + _sat_line(11, 2) + _sat_line(11, 3)
    ).encode())
    c._on_stat(msg3, node, "/stat")
    assert len(c.snapshot()["stat"]["sats"]) == 3


def test_stat_stream_survives_chunk_boundary():
    c = _collector()
    node = FakeNode()
    c._on_stat(types.SimpleNamespace(data=b"$SAT,10,10,G01,1,12"), node, "/stat")
    c._on_stat(types.SimpleNamespace(data=b"0,45,0.1,1,1,40,0,0,0,0,0,0\n"), node, "/stat")
    assert len(c.snapshot()["stat"]["sats"]) == 1


# --- T4: diagnostics -------------------------------------------------------
def _diag_msg(level, message, values):
    kv = [types.SimpleNamespace(key=k, value=v) for k, v in values.items()]
    return types.SimpleNamespace(status=[types.SimpleNamespace(
        name="gnss_diag", level=level, message=message, values=kv)])


def test_diag_level_mapping_and_fields():
    st = diag_state(_diag_msg(1, "差分中断", {
        "status_code": "corr_outage", "open_events": "corr_outage,low_sats",
        "divergence_m": "1.234", "verdict.corr_outage": "warning 差分中断",
        "verdict.low_sats": "ok 卫星数正常",
    }), 1.0)
    assert st["level_name"] == "warning"
    assert st["status_code"] == "corr_outage"
    assert st["open_events"] == ["corr_outage", "low_sats"]
    assert st["divergence_m"] == 1.234
    assert st["verdict"]["corr_outage"]["level"] == "warning"
    assert st["verdict"]["corr_outage"]["message"] == "差分中断"


def test_diag_open_events_empty_dash():
    st = diag_state(_diag_msg(0, "ok", {"open_events": "-"}), 1.0)
    assert st["open_events"] == [] and st["level_name"] == "ok"


def test_diag_worst_status_wins():
    statuses = [
        types.SimpleNamespace(name="a", level=0, message="ok", values=[]),
        types.SimpleNamespace(name="b", level=2, message="err", values=[]),
    ]
    st = diag_state(types.SimpleNamespace(status=statuses), 1.0)
    assert st["level"] == 2 and st["level_name"] == "error"


def test_collector_keeps_both_diagnostic_sources():
    c = _collector()
    node = FakeNode()
    c._on_diagnostic("gnss", _diag_msg(1, "warn", {}), node, "/gnss/dig")
    node.clock.advance(1.0)
    c._on_diagnostic("rtkrcv", _diag_msg(0, "ok", {}), node, "/rtkrcv/dig")
    snap = c.snapshot()
    assert snap["diagnostics"]["gnss"]["level_name"] == "warning"
    assert snap["diagnostics"]["rtkrcv"]["level_name"] == "ok"


# --- rates ----------------------------------------------------------------
def test_rate_bytes_per_second_window():
    c = _collector()
    node = FakeNode()
    for i in range(5):
        c._on_rate("corr", types.SimpleNamespace(data=b"x" * 100), node, "/r")
        node.clock.advance(1.0)
    r = c.snapshot()["rates"]["corr"]
    assert r["bytes"] == 100
    # 5 messages over a 4 s span -> 500 B / 4 s
    assert r["rate_bps"] == 125.0


def test_rate_outage_after_silence():
    c = _collector()
    node = FakeNode()
    c._on_rate("obs", types.SimpleNamespace(data=b"x" * 10), node, "/r")
    node.clock.advance(10.0)
    r = c.snapshot()["rates"]["obs"]
    assert r["last_t"] == 100.0
