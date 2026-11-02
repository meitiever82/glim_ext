"""T5: status aggregation from the read-only ROS snapshot."""
import types

import pytest

from gnss_hmi.collectors.ros_collector import RosCollector, TopicCfg
from gnss_hmi.config import load_config
from gnss_hmi.main import build_app

_YAML = """
data_root: {root}
db_path: {root}/hmi.db
storage: {{enabled: false}}
can: {{enabled: false, channel: can0}}
corrections: {{enabled: false, host: 127.0.0.1, port: 1}}
raw_obs: {{enabled: false, host: 127.0.0.1, port: 1}}
gnss_solution: {{enabled: false, host: 127.0.0.1, port: 0, listen: false}}
can_channel: can0
reserve: {{corrections_port: 0, raw_obs_port: 0}}
retention_days: 14
disk_watermark_pct: 85.0
diagnosis: {{local_fallback: false}}
ros:
  enabled: {ros}
  topics:
    - {{topic: /rtkrcv_node/rtk_fix, kind: rtkfix, source: app, qos: reliable}}
    - {{topic: /gnss_cgi610/rtk_fix, kind: rtkfix, source: fused}}
    - {{topic: /rtkrcv_node/stat, kind: stat}}
    - {{topic: /gnss/diagnostics, kind: diagnostic, source: gnss}}
    - {{topic: /rtkrcv_node/diagnostics, kind: diagnostic, source: rtkrcv}}
    - {{topic: /gnss/rtcm_corrections, kind: rate, name: corr}}
"""


class _Clock:
    def __init__(self, t=100.0):
        self.t = t

    def now(self):
        return types.SimpleNamespace(nanoseconds=int(self.t * 1e9))


class _Node:
    def __init__(self):
        self.clock = _Clock()

    def get_clock(self):
        return self.clock


def _rtk(q=4, sats=20, lat=44.5, heading=90.0, valid=True):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=100, nanosec=0)),
        gnss_time=0.0, quality=q, raw_status=q, latitude=lat, longitude=90.2,
        altitude=1200.0, sigma_enu=[0.01, 0.02, 0.03], diff_age=1.0,
        sats_used=sats, sats_main=sats, sats_aux=0,
        heading=heading, heading_sigma=0.1, heading_valid=valid, ratio=0.0)


def _diag(level=0, message="ok", open_events="-"):
    values = [
        types.SimpleNamespace(key="status_code", value="ok"),
        types.SimpleNamespace(key="open_events", value=open_events),
        types.SimpleNamespace(key="divergence_m", value="-"),
    ]
    return types.SimpleNamespace(status=[types.SimpleNamespace(
        name="gnss_diag", level=level, message=message, values=values)])


def _make_app(tmp_path):
    p = tmp_path / "cfg.yaml"
    p.write_text(_YAML.format(root=tmp_path, ros="true"))
    return build_app(load_config(p))


def _collector():
    return RosCollector("ros_link", (), lambda e: None, lambda *a: None)


def test_status_contains_both_sources(tmp_path):
    app = _make_app(tmp_path)
    c = _collector()
    node = _Node()
    c._on_rtkfix("app", _rtk(q=3, sats=9), node, "/app")
    node.clock.t += 0.5
    c._on_rtkfix("fused", _rtk(q=4, sats=21), node, "/fused")
    app._ros_collector = c
    app._diagnosis_tick()
    st = app.last_status
    assert st["sources"]["app"]["q"] == 3
    assert st["sources"]["fused"]["q"] == 4
    assert st["sources"]["app"]["stale"] is False
    assert st["sources"]["fused"]["sats"] == 21
    assert st["stale"] is False


def test_missing_one_source_keeps_the_other(tmp_path):
    app = _make_app(tmp_path)
    c = _collector()
    c._on_rtkfix("fused", _rtk(4, 21), _Node(), "/fused")
    app._ros_collector = c
    app._diagnosis_tick()
    st = app.last_status
    assert st["sources"]["app"] is None
    assert st["sources"]["fused"]["q"] == 4


def test_stale_after_three_seconds(tmp_path):
    app = _make_app(tmp_path)
    c = _collector()
    node = _Node()
    c._on_rtkfix("app", _rtk(), node, "/app")
    node.clock.t += 4.0
    app._ros_collector = c
    app._diagnosis_tick()
    st = app.last_status
    assert st["sources"]["app"]["stale"] is True
    assert st["stale"] is True


def test_verdict_comes_from_ros_diagnostics(tmp_path):
    app = _make_app(tmp_path)
    c = _collector()
    c._on_diagnostic("gnss", _diag(1, "差分中断", "corr_outage"), _Node(), "/gnss/dig")
    app._ros_collector = c
    app._diagnosis_tick()
    assert app.last_status["verdict"]["level"] == "warning"
    assert app.last_status["verdict"]["message"] == "差分中断"


def test_open_event_emits_ws_event_once(tmp_path):
    app = _make_app(tmp_path)
    c = _collector()
    c._on_diagnostic("gnss", _diag(1, "差分中断", "corr_outage"), _Node(), "/gnss/dig")
    app._ros_collector = c
    q = app.broadcaster.subscribe()
    app._diagnosis_tick()
    msgs = []
    while not q.empty():
        msgs.append(q.get_nowait())
    events = [m for m in msgs if m["type"] == "event"]
    assert len(events) == 1 and events[0]["action"] == "open"
    # same open_events on the next tick -> no duplicate
    app._diagnosis_tick()
    more = []
    while not q.empty():
        more.append(q.get_nowait())
    assert not [m for m in more if m["type"] == "event"]


def test_event_closes_when_open_events_clears(tmp_path):
    app = _make_app(tmp_path)
    c = _collector()
    c._on_diagnostic("gnss", _diag(1, "差分中断", "corr_outage"), _Node(), "/gnss/dig")
    app._ros_collector = c
    q = app.broadcaster.subscribe()
    app._diagnosis_tick()
    c._on_diagnostic("gnss", _diag(0, "ok", "-"), _Node(), "/gnss/dig")
    app._diagnosis_tick()
    msgs = []
    while not q.empty():
        msgs.append(q.get_nowait())
    closes = [m for m in msgs if m["type"] == "event" and m["action"] == "close"]
    assert len(closes) == 1 and closes[0]["event"]["code"] == "corr_outage"


def test_sat_stream_feeds_skyplot_json(tmp_path):
    app = _make_app(tmp_path)
    c = _collector()
    node = _Node()
    line = "$SAT,10,10,G01,1,120,45,0.1,1,1,42,0,0,0,0,0,0\n"
    c._on_stat(types.SimpleNamespace(data=line.encode()), node, "/stat")
    app._ros_collector = c
    app._diagnosis_tick()
    import json
    sats = json.loads(app.last_status["sky"])
    assert len(sats) == 1 and sats[0]["sat"] == "G01"


def test_no_ros_data_does_not_crash(tmp_path):
    app = _make_app(tmp_path)
    app._diagnosis_tick()
    st = app.last_status
    assert st["sources"]["app"] is None and st["sources"]["fused"] is None
    assert st["stale"] is True
    assert st["verdict"]["code"] == "waiting"
