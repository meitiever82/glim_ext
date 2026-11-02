# tests/test_api.py — TestClient 覆盖 REST 与 WS 回放（live 推送在 Task 10 端到端验证）
import dataclasses
import types

from fastapi.testclient import TestClient

from gnss_hmi.storage.epochs import Epoch


class _FakeApp:
    """Minimal stand-in exposing exactly what create_api consumes."""
    def __init__(self, tmp_path):
        from gnss_hmi.broadcast import Broadcaster
        from gnss_hmi.config import load_config  # not used; cfg faked below
        from gnss_hmi.diagnosis.base_station import BaseStationMonitor
        from gnss_hmi.storage.epochs import EpochStore
        from gnss_hmi.storage.events import EventStore
        self.epochs = EpochStore(tmp_path / "a.db")
        self.events = EventStore(tmp_path / "a.db")
        self.broadcaster = Broadcaster()
        self.base_monitor = BaseStationMonitor(self.epochs, warmup_s=1.0)
        self.last_status = {"type": "status", "t": 123.0}
        self.tile_store = None
        from gnss_hmi.config import DiagnosisCfg
        self.cfg = types.SimpleNamespace(
            diagnosis=DiagnosisCfg(), control_points=(),
            web=types.SimpleNamespace(static_dir=""))


def _client(tmp_path):
    from gnss_hmi.api import create_api
    fake = _FakeApp(tmp_path)
    return TestClient(create_api(fake)), fake


def test_status_events_epochs(tmp_path):
    c, fake = _client(tmp_path)
    fake.epochs.add(Epoch(t=100.0, src="can", q=4, lat=44.5, lon=90.2))
    fake.events.record(100.0, "diagnosis", "open", "x", level="serious", code="corr_outage")
    assert c.get("/api/status").json()["t"] == 123.0
    evs = c.get("/api/events").json()
    assert evs[0]["code"] == "corr_outage"
    eps = c.get("/api/epochs", params={"src": "can", "t0": 0, "t1": 200}).json()
    assert eps[0]["lat"] == 44.5


def test_epochs_endpoint_returns_newest_limit_ordered_ascending(tmp_path):
    """I5: full materialization + Python [-limit:] slicing replaced by
    EpochStore.query_last; the endpoint must return exactly `limit` newest
    rows, oldest-first."""
    c, fake = _client(tmp_path)
    for i in range(20):
        fake.epochs.add(Epoch(t=float(i), src="can", lat=float(i)))
    eps = c.get("/api/epochs", params={"src": "can", "t0": 0, "t1": 100, "limit": 5}).json()
    assert [e["t"] for e in eps] == [15.0, 16.0, 17.0, 18.0, 19.0]


def test_epochs_endpoint_clamps_limit_to_1_50000(tmp_path):
    c, fake = _client(tmp_path)
    for i in range(5):
        fake.epochs.add(Epoch(t=float(i), src="can"))
    # limit=0 clamps up to 1
    eps = c.get("/api/epochs", params={"src": "can", "t0": 0, "t1": 100, "limit": 0}).json()
    assert len(eps) == 1 and eps[0]["t"] == 4.0
    # limit far above 50000 clamps down (only 5 rows exist, so this just
    # confirms the request doesn't error and still returns everything)
    eps2 = c.get("/api/epochs", params={"src": "can", "t0": 0, "t1": 100, "limit": 999999}).json()
    assert len(eps2) == 5


def test_base_reset_needs_history(tmp_path):
    c, fake = _client(tmp_path)
    assert c.post("/api/base_reset").status_code == 409
    fake.epochs.add_base(100.0, 1.0, 2.0, 3.0)
    r = c.post("/api/base_reset")
    assert r.status_code == 200 and r.json()["xyz"] == [1.0, 2.0, 3.0]


def test_report_json_and_html(tmp_path):
    c, fake = _client(tmp_path)
    fake.epochs.add(Epoch(t=100.0, src="rtkrcv", q=1))
    assert c.get("/api/report", params={"t0": 0, "t1": 200}).json()["fix_ratio"] == 1.0
    html = c.get("/report", params={"t0": 0, "t1": 200}).text
    assert "固定解可用率" in html


def test_report_html_escaping(tmp_path):
    c, fake = _client(tmp_path)
    fake.epochs.add(Epoch(t=100.0, src="rtkrcv", q=1))
    # Record an event with potentially malicious content in code, level, and message.
    # etype="diagnosis": compute_report now filters to diagnosis rows only (C2).
    fake.events.record(100.0, "diagnosis", "open", "detail", level="<script>alert(1)</script>",
                       code="<img src=x onerror=alert(1)>")
    html_text = c.get("/report", params={"t0": 0, "t1": 200}).text
    # Assert that raw script tags are not in the HTML
    assert "<script>" not in html_text
    assert "<img src=" not in html_text
    # Assert that the escaped versions are present
    assert "&lt;script&gt;" in html_text
    assert "&lt;img src=" in html_text


def test_report_html_shows_zero_duration_not_dash(tmp_path):
    """e['duration_s'] or '-' rendered a real 0.0 duration as '-' (0.0 is
    falsy). Only an actually-missing duration_s (still-open event) should
    show the dash."""
    c, fake = _client(tmp_path)
    fake.epochs.add(Epoch(t=100.0, src="rtkrcv", q=1))
    rid = fake.events.record(100.0, "diagnosis", "open", "x", level="serious", code="corr_outage")
    fake.events.close_event(rid, 100.0)              # duration_s == 0.0
    html_text = c.get("/report", params={"t0": 0, "t1": 200}).text
    assert "<td>0.0</td>" in html_text
    assert "corr_outage</td><td>serious</td><td>-</td>" not in html_text


def test_tiles_404_without_store(tmp_path):
    c, _ = _client(tmp_path)
    assert c.get("/tiles/2/1/1.png").status_code == 404


def test_tiles_z_out_of_range_404s_without_touching_store(tmp_path):
    c, fake = _client(tmp_path)

    class _Stub:
        def get(self, z, x, y):
            raise AssertionError("store must not be queried for an out-of-range z")
    fake.tile_store = _Stub()
    assert c.get("/tiles/26/1/1.png").status_code == 404
    assert c.get("/tiles/-1/1/1.png").status_code == 404


def test_tiles_z_in_range_reaches_store(tmp_path):
    c, fake = _client(tmp_path)

    class _Stub:
        def get(self, z, x, y):
            assert 0 <= z <= 25
            return b"PNGDATA"
    fake.tile_store = _Stub()
    r = c.get("/tiles/10/1/1.png")
    assert r.status_code == 200 and r.content == b"PNGDATA"


def test_tiles_info_unavailable_without_store(tmp_path):
    c, fake = _client(tmp_path)
    info = c.get("/api/tiles_info").json()
    assert info["available"] is False
    assert info["version"] is None


def test_tiles_info_available_with_store(tmp_path):
    c, fake = _client(tmp_path)

    class _Stub:
        def get(self, z, x, y):
            raise AssertionError("tiles_info must not query the store")
    fake.tile_store = _Stub()
    info = c.get("/api/tiles_info").json()
    assert info["available"] is True


def test_pos_export(tmp_path):
    c, fake = _client(tmp_path)
    fake.epochs.add(Epoch(t=1000.5, src="app", q=4, sats=20, lat=44.5, lon=90.3,
                          alt=100.0, sdn=0.01, sde=0.02, sdu=0.03, age=1.0, ratio=3.5))
    r = c.get("/api/pos", params={"src": "app", "t0": 1000, "t1": 1001})
    assert r.status_code == 200
    assert r.text.startswith("% program: gnss-hmi")
    assert "44.500000000" in r.text
    assert "1970/01/01 00:16:40.500" in r.text   # UTC from epoch t, not dump time
    assert "attachment" in r.headers["content-disposition"]


def test_pos_rejects_bad_range(tmp_path):
    c, _ = _client(tmp_path)
    for t0, t1 in ((1, 1), (2, 1), (float("nan"), 1)):
        r = c.get("/api/pos", params={"src": "app", "t0": t0, "t1": t1})
        assert r.status_code == 400


def test_ws_replay_roundtrip(tmp_path):
    c, fake = _client(tmp_path)
    fake.epochs.add(Epoch(t=100.5, src="can", q=4, lat=44.5, lon=90.2, heading=170.0, speed=5.0))
    with c.websocket_connect("/ws") as ws:
        ws.send_json({"cmd": "replay", "t0": 100.0, "t1": 101.0, "speed": 1000.0})
        kinds = []
        while True:
            m = ws.receive_json()
            kinds.append(m["type"])
            if m["type"] == "replay_end":
                break
        assert "position" in kinds and "status" in kinds


def test_ws_replay_missing_keys_sends_error_and_keeps_live(tmp_path):
    c, fake = _client(tmp_path)
    with c.websocket_connect("/ws") as ws:
        ws.send_json({"cmd": "replay"})                    # no t0/t1 at all
        m = ws.receive_json()
        assert m == {"type": "error", "detail": "invalid replay command"}
        # live must still be running -- a broadcast reaches this client.
        fake.broadcaster.publish({"type": "status", "t": 1.0})
        m2 = ws.receive_json()
        assert m2 == {"type": "status", "t": 1.0}


def test_ws_replay_rejects_t1_before_t0(tmp_path):
    c, fake = _client(tmp_path)
    with c.websocket_connect("/ws") as ws:
        ws.send_json({"cmd": "replay", "t0": 100.0, "t1": 50.0})
        m = ws.receive_json()
        assert m["type"] == "error"
        fake.broadcaster.publish({"type": "status", "t": 2.0})
        assert ws.receive_json()["type"] == "status"


def test_ws_replay_rejects_nan(tmp_path):
    c, fake = _client(tmp_path)
    with c.websocket_connect("/ws") as ws:
        ws.send_json({"cmd": "replay", "t0": float("nan"), "t1": 101.0})
        m = ws.receive_json()
        assert m["type"] == "error"
        fake.broadcaster.publish({"type": "status", "t": 3.0})
        assert ws.receive_json()["type"] == "status"


def test_ws_replay_rejects_non_positive_speed(tmp_path):
    c, fake = _client(tmp_path)
    with c.websocket_connect("/ws") as ws:
        ws.send_json({"cmd": "replay", "t0": 0.0, "t1": 101.0, "speed": 0.0})
        m = ws.receive_json()
        assert m["type"] == "error"


def test_ws_replay_clamps_window_over_48h(tmp_path, monkeypatch):
    """{"cmd":"replay","t1":9e15} used to build one status entry per second
    over the whole [t0, t1] range synchronously before any row was read --
    billions of entries for an unclamped window. The ws handler must clamp
    t0 to at most 48h before t1 regardless of what the client sent."""
    c, fake = _client(tmp_path)
    import gnss_hmi.api as api_mod
    captured = {}

    async def fake_replay(epochs, events, t0, t1, speed=1.0):
        captured["t0"], captured["t1"] = t0, t1
        yield {"type": "replay_end", "t": t1}

    monkeypatch.setattr(api_mod, "replay_messages", fake_replay)
    with c.websocket_connect("/ws") as ws:
        ws.send_json({"cmd": "replay", "t0": 0.0, "t1": 9e15, "speed": 1000.0})
        m = ws.receive_json()
        assert m["type"] == "replay_end"
    assert captured["t1"] - captured["t0"] <= 172800.0
    assert captured["t0"] == captured["t1"] - 172800.0


def test_ws_replay_error_is_observed_and_live_recovers(tmp_path, monkeypatch):
    """I7: run_replay's body used to be unguarded -- a DB/socket error while
    iterating replay_messages would just vanish (the task's exception is
    never retrieved) and live would never restart. The handler must send an
    error message best-effort and hand the client back to live."""
    c, fake = _client(tmp_path)
    import gnss_hmi.api as api_mod

    async def boom(*a, **k):
        raise RuntimeError("db exploded")
        yield  # pragma: no cover -- unreachable; keeps this an async generator

    monkeypatch.setattr(api_mod, "replay_messages", boom)
    with c.websocket_connect("/ws") as ws:
        ws.send_json({"cmd": "replay", "t0": 0.0, "t1": 1.0, "speed": 1.0})
        m = ws.receive_json()
        assert m == {"type": "error", "detail": "replay failed"}
        # live must have been restarted -- drive it via the broadcaster, the
        # same path a real diagnosis tick/collector uses.
        fake.broadcaster.publish({"type": "status", "t": 42.0})
        m2 = ws.receive_json()
        assert m2 == {"type": "status", "t": 42.0}


def test_static_dir_configurable(tmp_path):
    from gnss_hmi.api import create_api
    from fastapi.testclient import TestClient
    d = tmp_path / "static"; d.mkdir()
    (d / "index.html").write_text("<h1>custom</h1>")
    fake = _FakeApp(tmp_path)

    class _W:  # duck cfg
        static_dir = str(d); host = "0.0.0.0"; port = 0; tiles_path = ""
    class _C:
        web = _W()
    fake.cfg = _C()
    c = TestClient(create_api(fake))
    assert "custom" in c.get("/").text
