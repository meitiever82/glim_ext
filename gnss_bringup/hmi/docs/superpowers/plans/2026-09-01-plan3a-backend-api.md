# Plan 3a: 后端 API 与回放 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** FastAPI/WebSocket 服务层、状态广播、历史回放引擎、报告数据、离线瓦片服务，以及 Plan 2 终审裁定携带的三项（数据库保留策略 + WAL、§4.3 事件峰值/关闭位置、gps_time 时间基准统一）。

**Architecture:** App 内新增 Broadcaster（asyncio 队列扇出）；诊断循环/CAN 回调把状态与位置消息推入；FastAPI（uvicorn 子任务，走 `_supervise`）提供 REST + WebSocket，静态目录留给 Plan 3b 前端。回放引擎从 SQLite 重建**与实时完全相同的消息格式**（spec §2 实时与回放同构）。

**Tech Stack:** FastAPI + uvicorn（新增运行时依赖，spec §2.1 api 模块既定）；测试新增 dev 依赖 httpx（TestClient）与 websockets（WS 客户端）。

**Spec:** `docs/superpowers/specs/2026-08-31-rtk-monitor-design.md`（§2.1 api/store、§4.3 事件字段补全、§6 回放与报告、§7 消息演进）。**前置**：Plan 2 分支（PR `plan2-solver`）合入 main 后执行本计划。

## Global Constraints

- 运行时依赖新增仅 `fastapi`、`uvicorn`；dev 新增仅 `httpx`、`websockets`。此前约束"仅 pyyaml/python-can"就此更新。
- 回放消息与实时消息**字段完全一致**（同构原则）；WS 消息契约见 Task 5，是 Plan 3b 的接口，字段名不可改。
- 采集永不停原则延续：web/回放任何异常不得影响采集与诊断（全部走 `_supervise` + 回调守卫）。
- 代码/注释英文；界面可见文案（报告 HTML）中文。TDD；每任务提交前全套测试绿（当前基线 94）。
- 每个 commit 末尾：`Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`。

## 文件结构（新增/修改总览）

```
src/rtk_monitor/
├── config.py          # 修改：WebCfg(port, tiles_path)、db_retention_days
├── storage/
│   ├── epochs.py      # 修改：WAL+busy_timeout、prune(before_t)
│   └── events.py      # 修改：WAL+busy_timeout、prune、迁移新列 lat_close/lon_close/peak、close_event 扩展
├── diagnosis/events.py# 修改：峰值指标跟踪、关闭位置
├── publisher.py       # 修改：两类消息补 host_time
├── broadcast.py       # 新增：Broadcaster（队列扇出，慢订阅者丢旧保新）
├── replay.py          # 新增：回放消息生成器
├── report.py          # 新增：报告统计纯函数
├── tiles.py           # 新增：MBTiles 读取
├── api.py             # 新增：FastAPI 应用工厂（REST+WS+静态+报告页+瓦片）
└── main.py            # 修改：broadcaster 接线、uvicorn 子任务、DB prune 接入清理循环
web/index.html         # 新增：占位页（Plan 3b 替换）
```

---

### Task 1: 依赖与配置（web 段 + 数据库保留天数）

**Files:**
- Modify: `pyproject.toml`、`config.yaml.example`、`src/rtk_monitor/config.py`
- Test: `tests/test_config.py`（追加）

**Interfaces:**
- Produces: `Config.web: WebCfg(port: int = 8080, tiles_path: str = "")`（yaml 段可选）；`Config.db_retention_days: int = 30`（顶层可选键）。pyproject 运行时依赖追加 `fastapi>=0.110`、`uvicorn>=0.29`；dev 追加 `httpx>=0.27`、`websockets>=12`。

- [ ] **Step 1: 写失败测试（追加）**

```python
def test_web_and_retention_defaults(tmp_path):
    text = "\n".join(l for l in EXAMPLE.read_text().splitlines()
                     if not l.startswith(("web", "db_retention_days"))
                     and not l.startswith(("  tiles_path",)))
    p = tmp_path / "c.yaml"; p.write_text(text)
    cfg = load_config(p)
    assert cfg.web.port == 8080 and cfg.web.tiles_path == ""
    assert cfg.db_retention_days == 30

def test_web_explicit():
    cfg = load_config(EXAMPLE)
    assert cfg.web.port == 8080          # example carries the section
```

- [ ] **Step 2: 运行确认失败** — `pytest tests/test_config.py -v`
- [ ] **Step 3: 实现**

config.py 追加：

```python
@dataclass(frozen=True)
class WebCfg:
    port: int = 8080
    tiles_path: str = ""
```

`Config` 加字段 `web: WebCfg`、`db_retention_days: int`；`load_config`：

```python
    w = raw.get("web") or {}
    ...
        web=WebCfg(port=int(w.get("port", 8080)),
                   tiles_path=str(w.get("tiles_path", ""))),
        db_retention_days=int(raw.get("db_retention_days", 30)),
```

pyproject：`dependencies` 追加 `"fastapi>=0.110", "uvicorn>=0.29"`；dev 追加 `"httpx>=0.27", "websockets>=12"`。config.yaml.example 追加：

```yaml
web:              # FastAPI/WebSocket UI service
  port: 8080
  tiles_path: ""  # MBTiles file for offline imagery; empty = grid fallback

db_retention_days: 30   # prune epochs/base_station/closed events older than this
```

- [ ] **Step 4: `pip install -e ".[dev]"` 后运行通过 + 全套**
- [ ] **Step 5: Commit** — `git add -A && git commit -m "feat: web config section, db retention setting, fastapi deps"`

---

### Task 2: 数据库 WAL 与保留清理

**Files:**
- Modify: `src/rtk_monitor/storage/epochs.py`、`src/rtk_monitor/storage/events.py`、`src/rtk_monitor/main.py`
- Test: `tests/test_epochs.py`、`tests/test_events.py`（各追加）

**Interfaces:**
- Produces: 两个 store 打开时执行 `PRAGMA journal_mode=WAL` 与 `PRAGMA busy_timeout=5000`（为 web 读并发做准备）。`EpochStore.prune(before_t: float) -> int`（删除 epochs 与 base_station 中 t<before_t 的行，返回删除数）；`EventStore.prune(before_t: float) -> int`（仅删 state='closed' 且 t<before_t 的行——开启中的事件永不删）。App `_cleanup_loop` 每小时调用两个 prune，`before_t = time.time() - cfg.db_retention_days*86400`，包在既有 try/except 内。

- [ ] **Step 1: 写失败测试（两处各追加）**

```python
# tests/test_epochs.py 追加
def test_prune_removes_old_rows(tmp_path):
    s = EpochStore(tmp_path / "e.db")
    s.add(Epoch(t=100.0, src="can")); s.add(Epoch(t=200.0, src="can"))
    s.add_base(100.0, 1, 2, 3); s.add_base(200.0, 1, 2, 3)
    n = s.prune(before_t=150.0)
    assert n == 2
    assert s.latest("can").t == 200.0 and len(s.base_history()) == 1

def test_wal_mode(tmp_path):
    s = EpochStore(tmp_path / "e.db")
    assert s._db.execute("PRAGMA journal_mode").fetchone()[0] == "wal"
```

```python
# tests/test_events.py 追加
def test_prune_keeps_open_events(tmp_path):
    s = EventStore(tmp_path / "e.db")
    rid = s.record(100.0, "diagnosis", "open", "x")
    s.record(100.0, "diagnosis", "open", "y")
    s.close_event(rid, 110.0)
    assert s.prune(before_t=200.0) == 1          # only the closed one
    assert [r.state for r in s.query()] == ["open"]
```

- [ ] **Step 2: 运行确认失败**
- [ ] **Step 3: 实现**

两个 store 的 `__init__` 建表后追加：

```python
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA busy_timeout=5000")
```

epochs.py：

```python
    def prune(self, before_t: float) -> int:
        cur = self._db.execute("DELETE FROM epochs WHERE t < ?", (before_t,))
        n = cur.rowcount
        n += self._db.execute("DELETE FROM base_station WHERE t < ?", (before_t,)).rowcount
        self._db.commit()
        return n
```

events.py：

```python
    def prune(self, before_t: float) -> int:
        cur = self._db.execute(
            "DELETE FROM events WHERE state='closed' AND t < ?", (before_t,))
        self._db.commit()
        return cur.rowcount
```

main.py `_cleanup_loop` 的 try 块内追加：

```python
                cutoff = time.time() - self.cfg.db_retention_days * 86400.0
                self.epochs.prune(cutoff)
                self.events.prune(cutoff)
```

- [ ] **Step 4: 运行通过 + 全套**
- [ ] **Step 5: Commit** — `git commit -m "feat: WAL mode and db retention pruning"`

---

### Task 3: §4.3 事件峰值指标与关闭位置

**Files:**
- Modify: `src/rtk_monitor/storage/events.py`、`src/rtk_monitor/diagnosis/events.py`、`src/rtk_monitor/main.py`
- Test: `tests/test_events.py`、`tests/test_event_machine.py`（各追加）

**Interfaces:**
- Produces: events 表迁移新列 `lat_close REAL, lon_close REAL, peak TEXT`（沿用 `_EXTRA_COLS` 机制），`EventRow` 加同名字段（默认 None）；`close_event(event_id, t_close, lat=None, lon=None, peak=None)` 扩展。`EventMachine.update(t, verdict, lat=None, lon=None, metrics=None)`：事件开启期间对 `metrics: dict[str, float]` 逐键取绝对值最大者累计为峰值；close 时写入 peak（JSON）与最后一次 lat/lon 作为关闭位置。App `_diagnosis_tick` 传 `metrics={"divergence_m": div_m or 0.0, "sats": float(sol.ns) if sol else 0.0, "corr_gap_s": now - corr_last_t if corr_last_t else 0.0}`。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_events.py 追加
def test_close_event_with_position_and_peak(tmp_path):
    s = EventStore(tmp_path / "e.db")
    rid = s.record(100.0, "diagnosis", "open", "x", code="corr_outage")
    s.close_event(rid, 130.0, lat=44.5, lon=90.2, peak='{"corr_gap_s": 12.0}')
    row = s.query()[0]
    assert row.lat_close == 44.5 and '"corr_gap_s"' in row.peak
```

```python
# tests/test_event_machine.py 追加
def test_peak_metrics_accumulate_and_persist(tmp_path):
    store = EventStore(tmp_path / "e.db")
    m = EventMachine(store, close_hysteresis_s=1.0)
    m.update(100.0, OUT, lat=44.0, lon=90.0, metrics={"corr_gap_s": 5.0})
    m.update(101.0, OUT, lat=44.1, lon=90.1, metrics={"corr_gap_s": 12.0})
    m.update(102.0, OK, metrics={"corr_gap_s": 0.0})
    m.update(104.0, OK)
    row = store.query()[0]
    import json
    assert row.state == "closed"
    assert json.loads(row.peak)["corr_gap_s"] == 12.0
    assert row.lat_close == 44.1 and row.lon_close == 90.1
```

- [ ] **Step 2: 运行确认失败**
- [ ] **Step 3: 实现**

events.py：`_EXTRA_COLS` 追加 `("lat_close","REAL"),("lon_close","REAL"),("peak","TEXT")`；`EventRow` 加三字段；`query` 的 SELECT 补三列；`close_event`：

```python
    def close_event(self, event_id: int, t_close: float, lat: float | None = None,
                    lon: float | None = None, peak: str | None = None) -> None:
        self._db.execute(
            "UPDATE events SET state='closed', t_close=?, lat_close=?, lon_close=?, peak=?"
            " WHERE id=?", (t_close, lat, lon, peak, event_id))
        self._db.commit()
```

diagnosis/events.py：`__init__` 加 `self._peak: dict[str, float] = {}`、`self._last_pos: tuple | None = None`；`update` 签名加 `metrics: dict[str, float] | None = None`，active 分支且事件开启期间：

```python
        if metrics:
            for k, v in metrics.items():
                if abs(v) > abs(self._peak.get(k, 0.0)):
                    self._peak[k] = v
        if lat is not None:
            self._last_pos = (lat, lon)
```

（open 时清空 `_peak`/`_last_pos` 再累计）；`_close` 改为：

```python
        import json
        peak = json.dumps(self._peak) if self._peak else None
        pos = self._last_pos or (None, None)
        self._store.close_event(event_id, t, lat=pos[0], lon=pos[1], peak=peak)
        self._peak = {}; self._last_pos = None
```

（保持既有"先复位状态后回调"的异常安全次序。）main.py `_diagnosis_tick` 的 `event_machine.update(...)` 调用补 `metrics=` 参数（按 Interfaces 给的三键，包在既有守卫内）。

- [ ] **Step 4: 运行通过 + 全套**
- [ ] **Step 5: Commit** — `git commit -m "feat: event peak metrics and close position per spec 4.3"`

---

### Task 4: gps_time 时间基准统一（host_time 双字段）

**Files:**
- Modify: `src/rtk_monitor/publisher.py`、`src/rtk_monitor/main.py`、spec §7 示例
- Test: `tests/test_publisher.py`（追加）

**Interfaces:**
- Produces: `publish_fix(sol, heading=None, host_time: float | None = None)` 与 `publish_event(kind, verdict, t)` 的消息各增加 `host_time` 字段（Unix 秒）：fix 的 `gps_time` 仍为 GPST（sol.t）、`host_time` 为解到达主机时刻（App 传 `self._sol_t`）；event 的 `gps_time` 与 `host_time` 同值（事件本就以主机时定时）。`ver` 保持 1（加法演进）。spec §7 的两个 JSON 示例同步补 `"host_time"` 字段。

- [ ] **Step 1: 写失败测试（追加）**

```python
async def test_host_time_present_on_both_message_types():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0)); rx.setblocking(False)
    p = UdpPublisher("127.0.0.1", rx.getsockname()[1])
    await p.start()
    sol = RtkSolution(t=1000.0, lat=1, lon=2, alt=3, q=1, ns=30,
                      sdn=.01, sde=.01, sdu=.03, age=.8, ratio=25.0)
    p.publish_fix(sol, heading=None, host_time=1234.5)
    p.publish_event("open", Verdict("serious", "corr_outage", "x"), 1000.5)
    await asyncio.sleep(0.05)
    loop = asyncio.get_running_loop()
    msgs = [json.loads((await asyncio.wait_for(loop.sock_recv(rx, 4096), 1.0)).decode())
            for _ in range(2)]
    fix = next(m for m in msgs if m["type"] == "gnss_fix")
    ev = next(m for m in msgs if m["type"] == "gnss_event")
    assert fix["host_time"] == 1234.5 and fix["gps_time"] == 1000.0
    assert ev["host_time"] == ev["gps_time"] == 1000.5
    await p.stop(); rx.close()
```

- [ ] **Step 2: 运行确认失败**
- [ ] **Step 3: 实现** — publisher.py：`publish_fix` 签名加 `host_time: float | None = None`，dict 加 `"host_time": host_time`；`publish_event` 的 dict 加 `"host_time": t`。main.py `_on_rtksol` 调用处补 `host_time=self._sol_t`。spec 文档 §7 两个示例各补 `"host_time":...` 字段并加一句时间基准说明（gps_time：fix 为 GPST、event 为主机时；host_time 统一主机 Unix 秒，跨流对齐用它）。
- [ ] **Step 4: 运行通过 + 全套**
- [ ] **Step 5: Commit** — `git commit -m "feat: host_time field unifies cross-stream time base (spec 7)"`

---

### Task 5: Broadcaster 与实时消息接线（WS 消息契约）

**Files:**
- Create: `src/rtk_monitor/broadcast.py`
- Modify: `src/rtk_monitor/main.py`
- Test: `tests/test_broadcast.py`

**Interfaces:**
- Produces: `Broadcaster(maxsize: int = 200)`：`subscribe() -> asyncio.Queue`、`unsubscribe(q)`、`publish(msg: dict)`（同步；慢订阅者队列满时丢最旧保最新，永不阻塞、永不抛）。App 增加 `self.broadcaster`，接线三类消息——**此即 Plan 3b 消费的 WS 消息契约，字段名冻结**：

```python
# 1 Hz，诊断循环末尾（epoch dict 用 dataclasses.asdict，None 字段保留）
{"type": "status", "t": now,
 "verdict": {"level": ..., "code": ..., "message": ...},
 "sol": {"t","q","ns","age","ratio","lat","lon","alt","sdn","sde","sdu"} | None,   # 陈旧性门后的 sol
 "can": <asdict(latest can epoch)> | None,
 "gpchc": <asdict(latest gpchc epoch)> | None,
 "corr": {"last_t": ..., "base_offset_m": ...}}
# ≤5 Hz，_on_can 内（NavCycle 直取，节流独立于历元 1Hz 节流）
{"type": "position", "t": cyc.host_time, "src": "can", "lat":..., "lon":...,
 "heading":..., "q": cyc.sat_status, "speed": cyc.vel[3]}
# 事件转换（与 UDP publisher 并行，来自 event_machine on_transition）
{"type": "event", "action": "open"|"close",
 "event": {"t":..., "level":..., "code":..., "message":...}}
```

- [ ] **Step 1: 写失败测试**

```python
# tests/test_broadcast.py
import asyncio
from rtk_monitor.broadcast import Broadcaster


async def test_fanout_and_slow_subscriber_drops_oldest():
    b = Broadcaster(maxsize=2)
    q1, q2 = b.subscribe(), b.subscribe()
    for i in range(4):
        b.publish({"n": i})
    assert q1.qsize() == 2 and (await q1.get())["n"] == 2   # oldest dropped
    assert (await q2.get())["n"] == 2
    b.unsubscribe(q1)
    b.publish({"n": 9})
    assert q1.qsize() == 1                                   # no longer fed
    assert (await q2.get())["n"] == 3 and (await q2.get())["n"] == 9


async def test_publish_with_no_subscribers_is_noop():
    Broadcaster().publish({"x": 1})
```

- [ ] **Step 2: 运行确认失败**
- [ ] **Step 3: 实现**

```python
# src/rtk_monitor/broadcast.py
"""Fan out realtime messages to WebSocket subscribers; never block, never raise."""
from __future__ import annotations

import asyncio


class Broadcaster:
    def __init__(self, maxsize: int = 200) -> None:
        self._maxsize = maxsize
        self._subs: set[asyncio.Queue] = set()

    def subscribe(self) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue(self._maxsize)
        self._subs.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue) -> None:
        self._subs.discard(q)

    def publish(self, msg: dict) -> None:
        for q in list(self._subs):
            try:
                q.put_nowait(msg)
            except asyncio.QueueFull:
                try:
                    q.get_nowait()          # drop oldest, keep newest
                except asyncio.QueueEmpty:
                    pass
                try:
                    q.put_nowait(msg)
                except asyncio.QueueFull:
                    pass
```

main.py：`__init__` 建 `self.broadcaster = Broadcaster()` 与 `self._last_pos_pub = 0.0`；`_diagnosis_tick` 末尾（守卫内）按契约组装 status 并 publish（sol 用**陈旧性门后的** `sol` 变量；epoch 转 dict 用 `dataclasses.asdict`）；`_on_can` 在历元写入逻辑旁加 ≤5Hz 位置发布（`cyc.host_time - self._last_pos_pub >= 0.2` 时 publish position）；`event_machine` 的 on_transition 回调在既有 UDP 发布旁追加 `self.broadcaster.publish({"type":"event","action":kind,"event":{"t":t,"level":verdict.level,"code":verdict.code,"message":verdict.message}})`。App 级验证并入 Task 10 端到端（本任务单元测试只覆盖 Broadcaster）。

- [ ] **Step 4: 运行通过 + 全套**
- [ ] **Step 5: Commit** — `git commit -m "feat: broadcaster and realtime message wiring (WS contract)"`

---

### Task 6: 回放引擎

**Files:**
- Create: `src/rtk_monitor/replay.py`
- Test: `tests/test_replay.py`

**Interfaces:**
- Consumes: EpochStore.query、EventStore.query。
- Produces: `async def replay_messages(epochs, events, t0: float, t1: float, speed: float = 1.0, sleep=asyncio.sleep)` — 异步生成器，按原时间轴（÷speed 的间隔，sleep 可注入为 no-op 供测试）产出**与 Task 5 契约完全同构**的消息：每个 can/gpchc 历元 → position 消息（src 对应）；每整秒 → status 消息（verdict 置 `{"level":"info","code":"replay","message":"回放"}`，sol/can/gpchc 取 ≤该秒最新历元的 asdict，corr 置 `{"last_t": None, "base_offset_m": None}`）；落在区间内的事件 → event 消息（open 用 t，closed 的关闭动作用 t_close）。结束后产出 `{"type": "replay_end", "t": t1}`。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_replay.py
from rtk_monitor.replay import replay_messages
from rtk_monitor.storage.epochs import Epoch, EpochStore
from rtk_monitor.storage.events import EventStore


async def _collect(gen):
    return [m async for m in gen]


async def test_replay_reconstructs_timeline(tmp_path):
    ep = EpochStore(tmp_path / "e.db"); ev = EventStore(tmp_path / "e.db")
    ep.add(Epoch(t=100.2, src="can", q=4, lat=44.5, lon=90.2, heading=170.0, speed=5.0))
    ep.add(Epoch(t=101.3, src="can", q=4, lat=44.6, lon=90.3, heading=171.0, speed=5.1))
    ep.add(Epoch(t=101.5, src="rtkrcv", q=1, sats=38, lat=44.6, lon=90.3))
    rid = ev.record(100.5, "diagnosis", "open", "差分中断", level="serious", code="corr_outage")
    ev.close_event(rid, 101.8)

    async def nosleep(_): pass
    msgs = await _collect(replay_messages(ep, ev, 100.0, 102.0, speed=10.0, sleep=nosleep))

    kinds = [m["type"] for m in msgs]
    assert kinds[-1] == "replay_end"
    assert kinds.count("position") == 2
    assert kinds.count("status") >= 2                       # one per whole second
    opens = [m for m in msgs if m["type"] == "event" and m["action"] == "open"]
    closes = [m for m in msgs if m["type"] == "event" and m["action"] == "close"]
    assert opens[0]["event"]["code"] == "corr_outage" and len(closes) == 1
    st = [m for m in msgs if m["type"] == "status"][-1]
    assert st["sol"]["q"] == 1 and st["can"]["heading"] == 171.0
    # ordering: messages non-decreasing in t
    ts = [m["t"] for m in msgs]
    assert ts == sorted(ts)
```

- [ ] **Step 2: 运行确认失败**
- [ ] **Step 3: 实现**

```python
# src/rtk_monitor/replay.py
"""Rebuild the realtime WS message stream from SQLite for a time range (spec §6)."""
from __future__ import annotations

import asyncio
import dataclasses
import math

from rtk_monitor.storage.epochs import EpochStore
from rtk_monitor.storage.events import EventStore

_SRC_POS = ("can", "gpchc")


async def replay_messages(epochs: EpochStore, events: EventStore, t0: float, t1: float,
                          speed: float = 1.0, sleep=asyncio.sleep):
    timeline: list[tuple[float, dict]] = []
    latest: dict[str, dict] = {}
    all_rows = {src: epochs.query(src, t0, t1) for src in ("rtkrcv", "can", "gpchc")}
    for src in _SRC_POS:
        for e in all_rows[src]:
            timeline.append((e.t, {"type": "position", "t": e.t, "src": src,
                                   "lat": e.lat, "lon": e.lon, "heading": e.heading,
                                   "q": e.q, "speed": e.speed}))
    for row in events.query(since=t0):
        edict = {"t": row.t, "level": row.level, "code": row.code, "message": row.detail}
        if t0 <= row.t <= t1:
            timeline.append((row.t, {"type": "event", "action": "open", "event": edict}))
        if row.t_close is not None and t0 <= row.t_close <= t1:
            timeline.append((row.t_close, {"type": "event", "action": "close",
                                           "event": dict(edict, t=row.t_close)}))
    for sec in range(math.ceil(t0), math.floor(t1) + 1):
        snap = {}
        for src in ("rtkrcv", "can", "gpchc"):
            rows = [e for e in all_rows[src] if e.t <= sec]
            snap[src] = dataclasses.asdict(rows[-1]) if rows else None
        timeline.append((float(sec), {
            "type": "status", "t": float(sec),
            "verdict": {"level": "info", "code": "replay", "message": "回放"},
            "sol": snap["rtkrcv"], "can": snap["can"], "gpchc": snap["gpchc"],
            "corr": {"last_t": None, "base_offset_m": None}}))
    timeline.sort(key=lambda x: x[0])
    prev = t0
    for t, msg in timeline:
        if t > prev:
            await sleep((t - prev) / max(speed, 0.01))
        prev = t
        yield msg
    yield {"type": "replay_end", "t": t1}
```

（注意：status 的 sol 取 rtkrcv 历元 asdict——字段是 Epoch 字段名，与实时 status 的 sol 键集不同属可接受差异？**不可**——同构原则。实时 status 的 sol 键为 {t,q,ns,age,ratio,lat,lon,alt,sdn,sde,sdu}，Epoch 的对应键为 {t,q,sats,age,ratio,lat,lon,alt,sde,sdn,sdu}。统一之：**实时侧 Task 5 组装 sol 时也用 epoch 风格键 `sats`**（即把 RtkSolution.ns 写成 "sats"、去掉实时/回放差异；Task 5 契约中的 sol 键集以本条为准修正为 {t,q,sats,age,ratio,lat,lon,alt,sdn,sde,sdu}）。实现 Task 5 时直接采用该键集。）

- [ ] **Step 4: 运行通过 + 全套**
- [ ] **Step 5: Commit** — `git commit -m "feat: replay engine reconstructing the realtime message stream"`

---

### Task 7: 报告统计纯函数

**Files:**
- Create: `src/rtk_monitor/report.py`
- Test: `tests/test_report.py`

**Interfaces:**
- Produces: `compute_report(epochs: EpochStore, events: EventStore, t0: float, t1: float) -> dict`，键：`fix_ratio`（rtkrcv 历元 q==1 占比，无 rtkrcv 历元时回退 can 历元 sat_status==4 占比，都无为 None）、`hourly`（list[{"hour": int_epoch_hour, "fix_ratio": float|None, "epochs": int}]）、`events`（list[{"code","level","t","t_close","duration_s","message"}]，按 t 排序）、`base_max_offset_m`（基站历史两两与首条的最大 3D 距离，空历史为 None）、`epoch_counts`（每 src 行数）。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_report.py
from rtk_monitor.report import compute_report
from rtk_monitor.storage.epochs import Epoch, EpochStore
from rtk_monitor.storage.events import EventStore


def test_report_stats(tmp_path):
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    base = 3600.0 * 100
    for i in range(10):
        ep.add(Epoch(t=base + i, src="rtkrcv", q=1 if i < 8 else 2))
    ep.add(Epoch(t=base + 3700, src="rtkrcv", q=1))
    rid = ev.record(base + 2, "diagnosis", "open", "差分中断", level="serious", code="corr_outage")
    ev.close_event(rid, base + 30)
    ep.add_base(base, -2148744.0, 4426641.0, 4044655.0)
    ep.add_base(base + 50, -2148744.5, 4426641.0, 4044655.0)

    r = compute_report(ep, ev, base, base + 7200)
    assert abs(r["fix_ratio"] - 9 / 11) < 1e-9
    assert r["hourly"][0]["epochs"] == 10 and abs(r["hourly"][0]["fix_ratio"] - 0.8) < 1e-9
    assert r["events"][0]["code"] == "corr_outage" and r["events"][0]["duration_s"] == 28.0
    assert abs(r["base_max_offset_m"] - 0.5) < 1e-6
    assert r["epoch_counts"]["rtkrcv"] == 11


def test_report_empty_range(tmp_path):
    ep = EpochStore(tmp_path / "r.db"); ev = EventStore(tmp_path / "r.db")
    r = compute_report(ep, ev, 0.0, 100.0)
    assert r["fix_ratio"] is None and r["events"] == [] and r["base_max_offset_m"] is None
```

- [ ] **Step 2: 运行确认失败**
- [ ] **Step 3: 实现**

```python
# src/rtk_monitor/report.py
"""Pure report statistics over stored epochs/events (spec §6)."""
from __future__ import annotations

import math

from rtk_monitor.storage.epochs import EpochStore
from rtk_monitor.storage.events import EventStore


def _fix_ratio(rows, fixed_q: int) -> float | None:
    if not rows:
        return None
    return sum(1 for e in rows if e.q == fixed_q) / len(rows)


def compute_report(epochs: EpochStore, events: EventStore, t0: float, t1: float) -> dict:
    rtk = epochs.query("rtkrcv", t0, t1)
    can = epochs.query("can", t0, t1)
    gpchc = epochs.query("gpchc", t0, t1)
    main_rows, fixed_q = (rtk, 1) if rtk else (can, 4)
    hourly = []
    if main_rows:
        h0, h1 = int(t0 // 3600), int(t1 // 3600)
        for h in range(h0, h1 + 1):
            rows = [e for e in main_rows if h * 3600 <= e.t < (h + 1) * 3600]
            hourly.append({"hour": h, "epochs": len(rows),
                           "fix_ratio": _fix_ratio(rows, fixed_q)})
    evs = []
    for r in events.query(since=t0):
        if r.t > t1:
            continue
        evs.append({"code": r.code, "level": r.level, "t": r.t, "t_close": r.t_close,
                    "duration_s": (r.t_close - r.t) if r.t_close else None,
                    "message": r.detail})
    hist = [h for h in epochs.base_history() if t0 <= h[0] <= t1]
    base_max = None
    if hist:
        x0, y0, z0 = hist[0][1:]
        base_max = max(math.dist((x0, y0, z0), h[1:]) for h in hist)
    return {"fix_ratio": _fix_ratio(main_rows, fixed_q), "hourly": hourly,
            "events": sorted(evs, key=lambda e: e["t"]),
            "base_max_offset_m": base_max,
            "epoch_counts": {"rtkrcv": len(rtk), "can": len(can), "gpchc": len(gpchc)}}
```

- [ ] **Step 4: 运行通过 + 全套**
- [ ] **Step 5: Commit** — `git commit -m "feat: report statistics"`

---

### Task 8: MBTiles 瓦片读取

**Files:**
- Create: `src/rtk_monitor/tiles.py`
- Test: `tests/test_tiles.py`

**Interfaces:**
- Produces: `TileStore(path: str)`：`get(z: int, x: int, y: int) -> bytes | None`（XYZ 输入，内部按 TMS 翻转 `row = 2**z - 1 - y` 查 `tiles` 表）、`close()`。`path` 为空或文件不存在时构造 `TileStore` 抛 `FileNotFoundError`（api 层据此禁用瓦片路由）。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_tiles.py
import sqlite3

import pytest

from rtk_monitor.tiles import TileStore


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
```

- [ ] **Step 2: 运行确认失败**
- [ ] **Step 3: 实现**

```python
# src/rtk_monitor/tiles.py
"""Read-only MBTiles access for the offline imagery basemap (spec §5)."""
from __future__ import annotations

import os
import sqlite3


class TileStore:
    def __init__(self, path: str) -> None:
        if not path or not os.path.isfile(path):
            raise FileNotFoundError(path)
        self._db = sqlite3.connect(f"file:{path}?mode=ro", uri=True,
                                   check_same_thread=False)

    def get(self, z: int, x: int, y: int) -> bytes | None:
        row = (2 ** z) - 1 - y
        cur = self._db.execute(
            "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
            (z, x, row)).fetchone()
        return bytes(cur[0]) if cur else None

    def close(self) -> None:
        self._db.close()
```

（`check_same_thread=False`：FastAPI 线程池可能跨线程调用；只读连接安全。）

- [ ] **Step 4: 运行通过 + 全套**
- [ ] **Step 5: Commit** — `git commit -m "feat: read-only MBTiles tile store"`

---

### Task 9: FastAPI 应用（REST + WS + 静态 + 报告页 + 瓦片）与 uvicorn 接线

**Files:**
- Create: `src/rtk_monitor/api.py`、`web/index.html`
- Modify: `src/rtk_monitor/main.py`
- Test: `tests/test_api.py`

**Interfaces:**
- Consumes: App 的 epochs/events/broadcaster/base_monitor/cfg、replay_messages、compute_report、TileStore。
- Produces: `create_api(app) -> FastAPI`（`app` 为 rtk_monitor 的 App 实例）。路由：
  - `GET /api/status` → 最近一次 status 消息（App 缓存 `self.last_status: dict | None`，Task 5 的 publish 处同步存一份）；无则 `{"type":"status","t":null}`。
  - `GET /api/events?since=0` → EventRow 列表（dataclasses.asdict）。
  - `GET /api/epochs?src=can&t0=..&t1=..&limit=3600` → 历元列表（asdict，`rows[-limit:]`）。
  - `GET /api/base_history` → [[t,x,y,z],...]。
  - `POST /api/base_reset` → 用 base_station 历史最新一条调 `base_monitor.reset`，返回 `{"ok": true, "xyz": [...]}`；无历史 409。
  - `GET /api/report?t0=..&t1=..` → compute_report JSON；`GET /report?t0=..&t1=..` → 可打印 HTML（内联样式，中文，纯服务端字符串模板：标题、fix_ratio 概览、每小时表格、事件表、基站稳定性一行）。
  - `GET /tiles/{z}/{x}/{y}.png` → TileStore 命中返回 `Response(data, media_type="image/png")`，未命中 404；`cfg.web.tiles_path` 无效时整个路由返回 404（App 构造 TileStore 失败置 None）。
  - `WS /ws`：连接即订阅 broadcaster 转发（live 模式）；收到 `{"cmd":"replay","t0":..,"t1":..,"speed":..}` 停 live、流式发送 replay_messages 至 replay_end 后自动回 live；收到 `{"cmd":"live"}` 中断回放回 live。断连时 unsubscribe。
  - 静态：`web/` 目录挂载到 `/`（html=True）；本任务的 index.html 为占位（标题 + "Plan 3b 前端待部署" + `/api/status` 链接）。
- main.py：App 增加 `self.last_status`、`self.tile_store`（构造失败置 None 并 log）、`self._web_server`（`uvicorn.Server(uvicorn.Config(create_api(self), host="0.0.0.0", port=cfg.web.port, log_level="warning"))`），`run_forever` 追加 `_supervise("web", self._web_server.serve)`；`shutdown` 置 `self._web_server.should_exit = True`。测试通过 `web_port()` 助手取实际端口（`self._web_server.servers[0].sockets[0].getsockname()[1]`，未启动返回 None）。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_api.py — TestClient 覆盖 REST 与 WS 回放（live 推送在 Task 10 端到端验证）
import dataclasses

from fastapi.testclient import TestClient

from rtk_monitor.storage.epochs import Epoch


class _FakeApp:
    """Minimal stand-in exposing exactly what create_api consumes."""
    def __init__(self, tmp_path):
        from rtk_monitor.broadcast import Broadcaster
        from rtk_monitor.config import load_config  # not used; cfg faked below
        from rtk_monitor.diagnosis.base_station import BaseStationMonitor
        from rtk_monitor.storage.epochs import EpochStore
        from rtk_monitor.storage.events import EventStore
        self.epochs = EpochStore(tmp_path / "a.db")
        self.events = EventStore(tmp_path / "a.db")
        self.broadcaster = Broadcaster()
        self.base_monitor = BaseStationMonitor(self.epochs, warmup_s=1.0)
        self.last_status = {"type": "status", "t": 123.0}
        self.tile_store = None


def _client(tmp_path):
    from rtk_monitor.api import create_api
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


def test_tiles_404_without_store(tmp_path):
    c, _ = _client(tmp_path)
    assert c.get("/tiles/2/1/1.png").status_code == 404


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
```

- [ ] **Step 2: 运行确认失败**
- [ ] **Step 3: 实现 api.py（核心骨架，完整实现按此展开）**

```python
# src/rtk_monitor/api.py
"""FastAPI service: REST, WebSocket (live + replay), report page, tiles, static UI."""
from __future__ import annotations

import asyncio
import dataclasses
from pathlib import Path

from fastapi import FastAPI, HTTPException, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from rtk_monitor.replay import replay_messages
from rtk_monitor.report import compute_report

_WEB_DIR = Path(__file__).resolve().parents[2] / "web"


def create_api(app) -> FastAPI:            # app: rtk_monitor.main.App (duck-typed)
    api = FastAPI(title="rtk-monitor")

    @api.get("/api/status")
    def status():
        return app.last_status or {"type": "status", "t": None}

    @api.get("/api/events")
    def events(since: float = 0.0):
        return [dataclasses.asdict(r) for r in app.events.query(since=since)]

    @api.get("/api/epochs")
    def epochs(src: str, t0: float, t1: float, limit: int = 3600):
        return [dataclasses.asdict(e) for e in app.epochs.query(src, t0, t1)[-limit:]]

    @api.get("/api/base_history")
    def base_history():
        return app.epochs.base_history()

    @api.post("/api/base_reset")
    def base_reset():
        hist = app.epochs.base_history()
        if not hist:
            raise HTTPException(409, "no base station history")
        t, x, y, z = hist[-1]
        app.base_monitor.reset(t, x, y, z)
        return {"ok": True, "xyz": [x, y, z]}

    @api.get("/api/report")
    def report_json(t0: float, t1: float):
        return compute_report(app.epochs, app.events, t0, t1)

    @api.get("/report", response_class=HTMLResponse)
    def report_html(t0: float, t1: float):
        r = compute_report(app.epochs, app.events, t0, t1)
        def pct(v):
            return "-" if v is None else f"{v:.1%}"

        rows = "".join(
            f"<tr><td>{h['hour']}</td><td>{h['epochs']}</td><td>{pct(h['fix_ratio'])}</td></tr>"
            for h in r["hourly"])
        evs = "".join(f"<tr><td>{e['code']}</td><td>{e['level']}</td>"
                      f"<td>{e['duration_s'] or '-'}</td><td>{e['message']}</td></tr>"
                      for e in r["events"])
        fr = "-" if r["fix_ratio"] is None else f"{r['fix_ratio']:.1%}"
        return f"""<html><meta charset="utf-8"><body style="font-family:sans-serif;max-width:800px;margin:2em auto">
<h1>RTK 定位报告</h1><p>固定解可用率：<b>{fr}</b>　基站最大偏移：{r['base_max_offset_m'] or '-'} m</p>
<h2>分小时统计</h2><table border=1 cellpadding=4><tr><th>小时</th><th>历元数</th><th>固定率</th></tr>{rows}</table>
<h2>事件（{len(r['events'])}）</h2><table border=1 cellpadding=4><tr><th>类型</th><th>级别</th><th>时长(s)</th><th>结论</th></tr>{evs}</table>
</body></html>"""

    @api.get("/tiles/{z}/{x}/{y}.png")
    def tile(z: int, x: int, y: int):
        if app.tile_store is None:
            raise HTTPException(404, "no tiles configured")
        data = app.tile_store.get(z, x, y)
        if data is None:
            raise HTTPException(404, "tile not found")
        return Response(data, media_type="image/png")

    @api.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        q = app.broadcaster.subscribe()
        replay_task: asyncio.Task | None = None

        async def live():
            while True:
                await sock.send_json(await q.get())

        async def run_replay(cmd):
            async for m in replay_messages(app.epochs, app.events,
                                           float(cmd["t0"]), float(cmd["t1"]),
                                           float(cmd.get("speed", 1.0))):
                await sock.send_json(m)

        live_task = asyncio.create_task(live())
        try:
            while True:
                cmd = await sock.receive_json()
                for t in (replay_task, ):
                    if t is not None:
                        t.cancel()
                if cmd.get("cmd") == "replay":
                    live_task.cancel()
                    replay_task = asyncio.create_task(run_replay(cmd))
                elif cmd.get("cmd") == "live":
                    if not live_task or live_task.done():
                        live_task = asyncio.create_task(live())
        except WebSocketDisconnect:
            pass
        finally:
            for t in (live_task, replay_task):
                if t is not None:
                    t.cancel()
            app.broadcaster.unsubscribe(q)

    api.mount("/", StaticFiles(directory=str(_WEB_DIR), html=True), name="web")
    return api
```

（web/index.html 占位内容：`<!doctype html><meta charset="utf-8"><title>rtk-monitor</title><h1>rtk-monitor</h1><p>Plan 3b 前端待部署。<a href="/api/status">/api/status</a></p>`。）main.py 接线按 Interfaces 描述实现。

- [ ] **Step 4: 运行通过 + 全套**
- [ ] **Step 5: Commit** — `git commit -m "feat: FastAPI service with WS live/replay, report page, tiles"`

---

### Task 10: 端到端（API + WS live + 回放）

**Files:**
- Test: `tests/test_e2e_api.py`

**Interfaces:** 消费 App 全装配 + fake rtkrcv（复用 tests/fake_rtkrcv.py）。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_e2e_api.py — full App with web enabled; raw websockets client
import asyncio
import json
import sys
import textwrap
from pathlib import Path

import websockets

from rtk_monitor.config import load_config
from rtk_monitor.main import build_app

FAKE = Path(__file__).parent / "fake_rtkrcv.py"


async def test_api_and_ws_live(tmp_path):
    cfg_file = tmp_path / "config.yaml"
    cfg_file.write_text(textwrap.dedent(f"""\
        data_root: {tmp_path}/log
        db_path: {tmp_path}/rtk.db
        corrections: {{host: 127.0.0.1, port: 1}}
        raw_obs: {{host: 127.0.0.1, port: 1}}
        gnss_solution: {{host: 127.0.0.1, port: 0, listen: true}}
        can_channel: virtual:apie2e
        reserve: {{corrections_port: 0, raw_obs_port: 0}}
        retention_days: 14
        disk_watermark_pct: 85.0
        rtkrcv: {{binary: "{sys.executable}", sol_port: 0, extra_args: ["{FAKE}"]}}
        web: {{port: 0}}
        diagnosis: {{corr_gap_s: 0.5}}
        """))
    app = build_app(load_config(cfg_file))
    task = asyncio.create_task(app.run_forever())
    for _ in range(100):
        await asyncio.sleep(0.1)
        if app.web_port() and app.last_status:
            break
    port = app.web_port()
    assert port

    import httpx
    async with httpx.AsyncClient() as c:
        st = (await c.get(f"http://127.0.0.1:{port}/api/status")).json()
        assert st["type"] == "status" and st["sol"]["q"] == 1
        eps = (await c.get(f"http://127.0.0.1:{port}/api/epochs",
                           params={"src": "rtkrcv", "t0": 0, "t1": 9e9})).json()
        assert eps and eps[-1]["sats"] == 38

    async with websockets.connect(f"ws://127.0.0.1:{port}/ws") as ws:
        for _ in range(20):
            m = json.loads(await asyncio.wait_for(ws.recv(), 3.0))
            if m["type"] == "status":
                assert m["verdict"]["code"] in ("corr_outage", "no_solution", "rtk_fixed",
                                                "not_fixed", "no_data")
                break
        else:
            raise AssertionError("no status message on live WS")
        await ws.send(json.dumps({"cmd": "replay", "t0": 0, "t1": 1, "speed": 1000}))
        while True:
            m = json.loads(await asyncio.wait_for(ws.recv(), 3.0))
            if m["type"] == "replay_end":
                break
    task.cancel()
    await app.shutdown()
```

- [ ] **Step 2: 运行确认失败**（web_port 助手/接线未就绪时）
- [ ] **Step 3: 补齐缺口**（预期主要是 main.py 的 `web_port()` 助手与启动顺序细节）
- [ ] **Step 4: 运行通过 + 全套（连跑 3 次确认稳定）**
- [ ] **Step 5: Commit** — `git commit -m "test: end-to-end API and websocket live/replay"`

---

## Self-Review 记录

- **Spec 覆盖**：§6 回放（T6/T9 WS 协议/T10）、§6 报告（T7/T9 HTML）、§2.1 api（T9）、§4.3 补全（T3）、§7 演进（T4）、瓦片服务端（T8/T9；瓦片**制作**与前端归 Plan 3b）、保留策略/WAL（T2）、基线 reset 界面入口的后端（T9 POST /api/base_reset）。§5 前端全部归 Plan 3b。
- **占位符扫描**：无 TBD/占位符。
- **类型一致性**：status 消息 sol 键集在 T6 修正为 epoch 风格（"sats"），T5 契约按此执行——两处已互相引用；Broadcaster/replay_messages/compute_report/TileStore 签名在 T9/T10 消费处一致；`web_port()`/`last_status`/`tile_store` 在 T9 定义、T10 消费。
- **风险注记**：TestClient 的 WS 测试走回放路径（同步端口）；live 推送靠 T10 真 uvicorn + websockets 客户端覆盖。回放 status 的 verdict 固定为 replay 占位（真实历史 verdict 未入库——如需真实回放结论，Plan 3b 可从 events 表叠加显示，已够用）。
