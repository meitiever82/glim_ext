"""ROS 只读适配层的纯逻辑测试（不需要 rclpy / ROS 环境）。

覆盖 spec 的映射与契约：
- RtkFix 五档 quality、sigma_enu 的 E/N/U 顺序、ratio/gnss_time 空值、NaN 拒绝
- 双源互不覆盖、旧解过期判定
- $SAT 跨块行重组与 tow 换历元
- DiagnosticArray 级别/open_events/verdict 解析
- 差分/观测字节速率窗口
"""
from __future__ import annotations

import math
import time
from types import SimpleNamespace

import pytest

from gnss_hmi.config import load_config
from gnss_hmi.ros_bridge import (LineAssembler, RateWindow, RosBridge,
                                 diag_values, parse_diagnostic_array,
                                 rtkfix_to_dict, unavailable, worst_of)

CFG_YAML = """\
data_root: {root}
db_path: {root}/hmi.db
corrections: {{host: 127.0.0.1, port: 1, enabled: false}}
raw_obs: {{host: 127.0.0.1, port: 1, enabled: false}}
gnss_solution: {{host: 127.0.0.1, port: 1, enabled: false}}
can_channel: can0
can_enabled: false
reserve: {{corrections_port: 0, raw_obs_port: 0}}
retention_days: 14
disk_watermark_pct: 85.0
storage: {{enabled: false}}
diagnosis: {{local_fallback: false}}
ros:
  enabled: true
  stale_s: 3.0
  topics:
    - {{topic: /rtkrcv_node/rtk_fix, kind: rtkfix, source: app, qos: reliable}}
    - {{topic: /gnss_cgi610/rtk_fix, kind: rtkfix, source: fused, qos: auto}}
    - {{topic: /rtkrcv_node/stat, kind: stat, qos: reliable}}
    - {{topic: /gnss/diagnostics, kind: diagnostic, qos: auto}}
    - {{topic: /rtkrcv_node/diagnostics, kind: diagnostic, qos: auto}}
    - {{topic: /gnss/rtcm_corrections, kind: rtcm_rate, qos: auto}}
    - {{topic: /gnss/raw_obs, kind: rtcm_rate, qos: auto}}
"""


def _cfg(tmp_path):
    p = tmp_path / "gnss_hmi.yaml"
    p.write_text(CFG_YAML.format(root=tmp_path))
    return load_config(p)


def _fix(**kw):
    base = dict(latitude=44.5, longitude=90.25, altitude=1000.0,
                sigma_enu=[0.010, 0.012, 0.030], diff_age=1.2,
                sats_used=24, sats_main=20, sats_aux=4, ratio=9.5,
                gnss_time=1730000000.0, quality=4, raw_status=0,
                heading=123.4, heading_valid=True,
                header=SimpleNamespace(stamp=SimpleNamespace(sec=1730000000, nanosec=50_000_000)))
    base.update(kw)
    return SimpleNamespace(**base)


def test_quality_zero_to_four_and_sigma_enu_order():
    snap = rtkfix_to_dict(_fix(quality=4), 100.0)
    assert snap["quality"] == 4 and snap["available"] is True
    # E/N/U 顺序：水平 σ = hypot(E, N)，不是拿 N/U 或旧 sdn/sde 命名
    assert snap["sigma_e"] == pytest.approx(0.010)
    assert snap["sigma_n"] == pytest.approx(0.012)
    assert snap["sigma_u"] == pytest.approx(0.030)
    assert snap["sigma_h"] == pytest.approx(math.hypot(0.010, 0.012))
    for q in (0, 1, 2, 3):
        assert rtkfix_to_dict(_fix(quality=q), 100.0)["quality"] == q


def test_ratio_and_gnss_time_zero_mean_not_provided():
    snap = rtkfix_to_dict(_fix(ratio=0.0, gnss_time=0.0), 100.0)
    assert snap["ratio"] is None
    assert snap["gnss_time"] is None
    assert snap["latency_s"] is None


def test_latency_uses_header_stamp_minus_gnss_time():
    snap = rtkfix_to_dict(_fix(), 100.0)
    assert snap["latency_s"] == pytest.approx(0.05)


def test_non_finite_coordinates_are_rejected():
    assert rtkfix_to_dict(_fix(latitude=float("nan")), 100.0) is None
    assert rtkfix_to_dict(_fix(longitude=float("inf")), 100.0) is None
    snap = rtkfix_to_dict(_fix(ratio=float("nan"), diff_age=float("nan")), 100.0)
    assert snap["ratio"] is None and snap["age_s"] is None


def test_invalid_heading_is_not_rendered():
    snap = rtkfix_to_dict(_fix(heading_valid=False), 100.0)
    assert snap["heading"] is None and snap["heading_valid"] is False


def test_dual_sources_do_not_overwrite_each_other():
    bridge = RosBridge(_cfg_stub())
    bridge._on_rtkfix("app", "/rtkrcv_node/rtk_fix", _fix(latitude=44.1))
    bridge._on_rtkfix("fused", "/gnss_cgi610/rtk_fix", _fix(latitude=44.2, quality=3))
    snap = bridge.snapshot()
    assert snap["sources"]["app"]["lat"] == pytest.approx(44.1)
    assert snap["sources"]["fused"]["lat"] == pytest.approx(44.2)
    assert snap["sources"]["fused"]["quality"] == 3


def _cfg_stub():
    from gnss_hmi.config import DEFAULT_ROS_TOPICS, RosCfg
    return RosCfg(topics=DEFAULT_ROS_TOPICS, stale_s=3.0)


def test_stale_source_keeps_last_fix_but_marks_stale():
    bridge = RosBridge(_cfg_stub())
    bridge._on_rtkfix("app", "/rtkrcv_node/rtk_fix", _fix())
    bridge._sources["app"]["t_recv"] = time.time() - 10
    snap = bridge.snapshot()
    src = snap["sources"]["app"]
    assert src["stale"] is True
    assert src["lat"] is not None          # 保留最后位置，不伪装成新解


def test_line_assembler_handles_split_lines():
    asm = LineAssembler()
    assert asm.feed(b"$SAT,1,10") == []
    out = asm.feed(b"0.0,G01\r\n$SAT,1,100.0,G02\n")
    assert out == ["$SAT,1,100.0,G01", "$SAT,1,100.0,G02"]
    assert asm.feed(b"") == []


def _sat_line(tow: float, sat: str, snr: float = 45.0) -> str:
    return (f"$SAT,1,{tow},{sat},0,45.0,30.0,1.2,0,1,{snr},0,0,0,3,0,0")


def test_sat_lines_group_into_latest_epoch_without_duplicate_freq():
    bridge = RosBridge(_cfg_stub())
    lines = [_sat_line(100.0, "G01"), _sat_line(100.0, "G01"), _sat_line(100.0, "G02")]
    for line in lines:
        bridge._sat_acc.feed(line)
    assert [s["sat"] for s in bridge._sat_acc.sats] == ["G01", "G02"]
    bridge._sat_acc.feed(_sat_line(101.0, "G03"))
    assert [s["sat"] for s in bridge._sat_acc.sats] == ["G03"]
    assert bridge._sat_acc.sats[0]["used"] is True


def _diag_status(name="gnss_diag", level=1, message="差分中断 4s", values=None):
    return SimpleNamespace(name=name, level=level, message=message, hardware_id="",
                           values=[SimpleNamespace(key=k, value=v)
                                   for k, v in (values or {}).items()])


def test_diagnostic_array_worst_level_and_fields():
    msg = SimpleNamespace(status=[
        _diag_status(level=0, message="OK", values={"status_code": "OK"}),
        _diag_status(level=2, message="定位不可用", values={
            "status_code": "NO_FIX", "open_events": "corr_outage,sat_low",
            "divergence_m": "0.42", "verdict.corr_outage": "ERROR|差分中断",
            "verdict.sat_low": "WARN|卫星不足"}),
    ])
    parsed = parse_diagnostic_array(msg, 100.0)
    assert parsed["level"] == 2 and parsed["level_name"] == "ERROR"
    assert parsed["message"] == "定位不可用"
    fields = diag_values(parsed)
    assert fields["status_code"] == "NO_FIX"
    assert fields["open_events"] == ["corr_outage", "sat_low"]
    assert fields["divergence_m"] == pytest.approx(0.42)
    assert fields["verdicts"]["corr_outage"] == "ERROR|差分中断"


class _NdArrayLike:
    """rclpy 的 float64[3] 是 ndarray：真值判断直接抛 ValueError，用它锁定
    “不能对定长数组写 `or []`”这条回归。"""

    def __init__(self, values):
        self._values = list(values)

    def __iter__(self):
        return iter(self._values)

    def __len__(self):
        return len(self._values)

    def __bool__(self):
        raise ValueError("The truth value of an array is ambiguous")


def test_rtkfix_sigma_accepts_ndarray_like():
    snap = rtkfix_to_dict(_fix(sigma_enu=_NdArrayLike([0.010, 0.012, 0.030])), 100.0)
    assert snap["sigma_e"] == pytest.approx(0.010)
    assert snap["sigma_n"] == pytest.approx(0.012)
    assert snap["sigma_u"] == pytest.approx(0.030)
    assert snap["sigma_h"] == pytest.approx(math.hypot(0.010, 0.012))


def test_diagnostic_level_from_octet_bytes():
    # DiagnosticStatus.level 在 rclpy 里是 octet → bytes(b'\x02')
    parsed = parse_diagnostic_array(
        SimpleNamespace(status=[_diag_status(level=b"\x02", message="定位不可用")]), 1.0)
    assert parsed["level"] == 2 and parsed["level_name"] == "ERROR"
    assert parsed["statuses"][0]["level"] == 2


def test_worst_of_merges_two_diagnostic_sources():
    ok = parse_diagnostic_array(SimpleNamespace(status=[_diag_status(level=0)]), 1.0)
    warn = parse_diagnostic_array(SimpleNamespace(status=[_diag_status(level=1, name="rtkrcv")]), 2.0)
    assert worst_of(ok, warn)["level_name"] == "WARN"
    assert worst_of(ok, None)["level_name"] == "OK"
    assert worst_of(None, None) is None


def test_diag_values_startup_grace_is_not_an_alarm():
    msg = SimpleNamespace(status=[
        _diag_status(level=0, message="启动宽限期", values={"status_code": "STARTUP_GRACE"})])
    fields = diag_values(parse_diagnostic_array(msg, 1.0))
    assert fields["startup_grace"] is True
    assert fields["open_events"] == []


def test_rate_window_and_idle():
    win = RateWindow(window_s=2.0)
    win.add(100.0, 1000)
    win.add(101.0, 1000)
    assert win.rate(101.0) == pytest.approx(2000.0)     # 2 帧 / 1 s
    assert win.idle_s(101.5) == pytest.approx(0.5)
    assert win.idle_s(103.0) == pytest.approx(2.0)
    win.add(104.0, 500)
    assert win.rate(104.0) == pytest.approx(500.0)      # 旧样本已滑出窗口


def test_link_status_reports_waiting_then_no_data():
    bridge = RosBridge(_cfg_stub())
    early = bridge.link_status(100.0, grace_end=110.0)
    assert all(link["state"] == "waiting" for link in early)
    late = bridge.link_status(111.0, grace_end=110.0)
    assert {link["state"] for link in late} == {"no_data"}


def test_stat_topic_chunks_are_assembled_into_satellites():
    bridge = RosBridge(_cfg_stub())
    chunk1 = (_sat_line(200.0, "G05") + "\n" + _sat_line(200.0, "G06")[:9]).encode()
    chunk2 = (_sat_line(200.0, "G06")[9:] + "\n").encode()
    bridge._on_stat("/rtkrcv_node/stat", SimpleNamespace(data=chunk1))
    bridge._on_stat("/rtkrcv_node/stat", SimpleNamespace(data=chunk2))
    snap = bridge.snapshot()
    assert [s["sat"] for s in snap["satellites"]] == ["G05", "G06"]
    assert snap["last_rx"]["/rtkrcv_node/stat"] > 0


def test_diag_topic_callbacks_keep_two_sources_apart():
    bridge = RosBridge(_cfg_stub())
    gnss = SimpleNamespace(status=[_diag_status(
        level=1, values={"status_code": "DEGRADED", "open_events": "sat_low"})])
    rtkrcv = SimpleNamespace(status=[_diag_status(
        name="rtkrcv", level=2, message="rtkrcv 退出")])
    bridge._on_diag(False, "/gnss/diagnostics", gnss)
    bridge._on_diag(True, "/rtkrcv_node/diagnostics", rtkrcv)
    snap = bridge.snapshot()
    assert snap["diag_gnss"]["level_name"] == "WARN"
    assert snap["diag_rtkrcv"]["level_name"] == "ERROR"
    assert diag_values(snap["diag_gnss"])["open_events"] == ["sat_low"]


def test_rate_windows_are_per_topic():
    bridge = RosBridge(_cfg_stub())
    bridge._on_bytes("/gnss/rtcm_corrections", SimpleNamespace(data=b"x" * 100))
    bridge._on_bytes("/gnss/raw_obs", SimpleNamespace(data=b"y" * 50))
    rates = bridge.snapshot()["rates"]
    assert set(rates) == {"/gnss/rtcm_corrections", "/gnss/raw_obs"}
    assert rates["/gnss/rtcm_corrections"][0] >= rates["/gnss/raw_obs"][0]


def test_split_verdict_parses_level_and_text():
    pytest.importorskip("fastapi")
    pytest.importorskip("uvicorn")
    from gnss_hmi.main import App
    assert App._split_verdict("WARN|差分中断") == ("warning", "差分中断")
    assert App._split_verdict("ERROR：无固定解") == ("critical", "无固定解")
    assert App._split_verdict("") == ("warning", "")


# --- 聚合层（需要 fastapi/uvicorn；缺失时跳过） ---------------------------
def test_ros_tick_builds_dual_source_status(tmp_path):
    pytest.importorskip("fastapi")
    pytest.importorskip("uvicorn")
    from gnss_hmi.main import App

    app = App(_cfg(tmp_path))
    assert app.bridge is not None and app.legacy_enabled is False

    class FakeBridge:
        def snapshot(self, now=None):
            now = time.time() if now is None else now
            app_src = rtkfix_to_dict(_fix(), now)
            fused = unavailable(reason="数据中断")
            return {"sources": {"app": app_src, "fused": fused},
                    "satellites": [{"sat": "G01", "az": 45.0, "el": 30.0,
                                    "snr": 45.0, "used": True}],
                    "satstats": [], "sat_t": now,
                    "diag_gnss": None, "diag_rtkrcv": None,
                    "rates": {}, "last_rx": {}, "ros_error": None}

        def link_status(self, now, grace_end):
            return []

    app.bridge = FakeBridge()
    app._ros_tick()
    st = app.last_status
    assert st["sources"]["app"]["available"] is True
    assert st["sources"]["fused"]["available"] is False
    assert st["sources"]["fused"]["reason"] == "无数据"   # 从未收到
    assert st["sat_count"] == 1
    assert st["stale"] is False
    assert set(st["corr_rate"]) >= {"rtcm_bps", "obs_bps", "rtcm_stale"}


def _app_with_bridge(tmp_path, bridge):
    pytest.importorskip("fastapi")
    pytest.importorskip("uvicorn")
    from gnss_hmi.main import App

    app = App(_cfg(tmp_path))
    app.bridge = bridge
    app._started_at = time.time() - 600.0      # 跳过硬限，让 stale 逻辑生效
    return app


def test_ros_tick_downgrades_when_diagnostics_go_stale(tmp_path):
    """诊断话题断流后不能继续把旧 WARN 当当前状态。"""
    now = time.time()

    class FakeBridge:
        def snapshot(self, now=None):
            return {"sources": {"app": rtkfix_to_dict(_fix(), now),
                                "fused": rtkfix_to_dict(_fix(), now)},
                    "satellites": [], "satstats": [], "sat_t": now,
                    # 10 s 前的诊断快照：已超 stale_s=3
                    "diag_gnss": {"t_recv": now - 10.0, "level": 1,
                                  "level_name": "WARN", "message": "差分中断",
                                  "statuses": []},
                    "diag_rtkrcv": None,
                    "rates": {}, "last_rx": {}, "ros_error": None}

        def link_status(self, now, grace_end):
            return []

    app = _app_with_bridge(tmp_path, FakeBridge())
    app._ros_tick()
    st = app.last_status
    assert st["sources"]["app"]["available"] is True
    assert st["verdict"]["level"] == "warning"
    assert "/gnss/diagnostics" in st["verdict"]["message"]
    assert "诊断中断" in st["verdict"]["message"]


def test_ros_open_events_are_recorded_and_closed(tmp_path):
    """事件要落到进程内事件表，页面刷新/WS 重连后仍有历史。"""
    app = _app_with_bridge(tmp_path, object())
    verdict = {"level": "warning", "code": "WARN_差分中断", "message": "差分中断",
               "open_events": ["CORR_AGE"],
               "verdicts": {"CORR_AGE": "WARN|差分中断"}, "startup_grace": False}
    app._publish_ros_events(verdict, 100.0)
    rows = app.events.query()
    assert [(r.etype, r.state, r.code, r.level) for r in rows] == [
        ("ros", "open", "CORR_AGE", "warning")]
    assert rows[0].detail == "差分中断"

    # 事件恢复后同一条记录被置为 closed，而不是再插一条
    app._publish_ros_events({"level": "ok", "code": "", "message": "",
                             "open_events": [], "verdicts": {}}, 200.0)
    rows = app.events.query()
    assert len(rows) == 1 and rows[0].state == "closed" and rows[0].t_close == 200.0
