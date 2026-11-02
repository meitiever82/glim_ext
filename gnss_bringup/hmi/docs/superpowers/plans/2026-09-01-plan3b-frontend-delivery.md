# Plan 3b: 前端与交付 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 深色中文前端 SPA（免构建 vendored Vue3 + Leaflet + uPlot，按 docs/ws-contract.md 消费冻结契约）、离线瓦片工具链文档、Docker/compose 交付（含 rtkrcv 编译），以及 Plan 3a 终审留下的四个后端携带项。

**Architecture:** 前端零构建：ESM 浏览器版 Vue3 + vendored Leaflet/uPlot 直接由 FastAPI 静态托管；组件为带模板字符串的小模块，纯逻辑集中在 `web/js/protocol.js`（node --test 可测）。地图轨迹用按固定状态分段的 Polyline；瓦片 404 时退化为坐标网格 GridLayer。Docker 双阶段：编 rtkrcv → python-slim 运行层。

**Tech Stack:** Vue 3.4 ESM（浏览器版，无编译）、Leaflet 1.9.4、uPlot 1.6.30（全部 vendored 进仓库）；node --test（本机 node v24，测纯逻辑）；Docker multi-stage。无新增 Python 依赖。

**Spec:** `docs/superpowers/specs/2026-08-31-rtk-monitor-design.md`（§5 前端界面、§9 部署）+ **`docs/ws-contract.md`（冻结的消息契约——前端一切字段名以它为准，含 corr 空值语义与故意的不对称）**。

## Global Constraints

- 前端不引入任何构建工具链（无 npm build/webpack/vite）；vendored 库文件直接提交，版本与来源记录在 `web/vendor/VENDOr.md`（文件名统一 `VENDOR.md`）。
- 界面中文、深色主题（spec §5）；代码/注释英文。
- WS/REST 字段名严格按 docs/ws-contract.md；`corr.last_t`/`base_offset_m` 为 null 表示"从未收到"，前端必须显示为"无数据"而非 0。
- 纯逻辑（颜色映射、格式化、轨迹分段决策）必须放 `web/js/protocol.js` 并有 node --test 覆盖；DOM/Leaflet/uPlot 胶水层不强制单测，但资产完整性与页面引用由 pytest 覆盖。
- Python 侧改动照旧 TDD；全套 pytest 保持绿（基线 154）。
- 每个 commit 末尾：`Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`。

## 文件结构（新增/修改总览）

```
src/rtk_monitor/
├── config.py          # 修改：web.host、web.static_dir
├── api.py             # 修改：静态目录可配置（容器内 pip install 后 parents[2] 失效）
├── main.py            # 修改：uvicorn host/graceful timeout、WAL checkpoint、sats_min 峰值
└── diagnosis/events.py# 修改：_min 后缀键用 min 聚合
web/
├── index.html         # 布局骨架（状态条/地图/右栏/时间线/回放条）
├── style.css          # 深色主题
├── app.js             # 启动：store + ws + 组件挂载
├── js/
│   ├── protocol.js    # 纯逻辑：颜色/徽章/格式化/分段（node 测试）
│   ├── store.js       # Vue reactive 状态
│   ├── ws.js          # WsClient（重连、live/replay 命令）
│   ├── statusbar.js   # 状态条组件
│   ├── eventlist.js   # 事件列表组件
│   ├── replaybar.js   # 回放控制组件
│   ├── mapview.js     # Leaflet 封装（瓦片/网格退化/三轨迹/箭头/σ圈）
│   ├── timeline.js    # uPlot ×4 封装
│   └── skyplot.js     # 天空图（占位态 + 有数据时渲染）
└── vendor/            # vue.esm-browser.prod.js、leaflet/、uplot/、VENDOR.md
tests_js/protocol.test.mjs
Dockerfile  docker-compose.yml  .dockerignore
docs/deploy.md（部署手册，中文）  docs/tiles-howto.md（瓦片制作，中文）
docs/integration-rtkrcv.md（追加 UI 联调项）
```

---

### Task 1: 后端携带项四合一（host/graceful/WAL checkpoint/sats_min + 静态目录可配置）

**Files:**
- Modify: `src/rtk_monitor/config.py`、`src/rtk_monitor/main.py`、`src/rtk_monitor/api.py`、`src/rtk_monitor/diagnosis/events.py`、`config.yaml.example`
- Test: `tests/test_config.py`、`tests/test_event_machine.py`、`tests/test_api.py`（各追加）

**Interfaces:**
- Produces: `WebCfg` 增加 `host: str = "0.0.0.0"`、`static_dir: str = ""`（空 = 沿用仓库相对 `web/`）。main.py：uvicorn Config 用 `cfg.web.host` 且加 `timeout_graceful_shutdown=5`；`_cleanup_loop` 在 prune 后对两个 store 各执行 `PRAGMA wal_checkpoint(TRUNCATE)`（store 各加方法 `checkpoint()`，异常吞并记日志）；`_diagnosis_tick` 的 metrics 键 `"sats"` 改为 `"sats_min"`。diagnosis/events.py：`_peak` 聚合规则——键名以 `_min` 结尾用 min（首见值直接记录），否则保持绝对值最大。api.py：`_WEB_DIR` 改为函数 `_web_dir(app)`：`getattr(app, "cfg", None)` 有 `web.static_dir` 非空则用之，否则回退仓库相对路径（FakeApp 无 cfg 也要能跑）。

- [ ] **Step 1: 写失败测试（三处追加）**

```python
# tests/test_config.py 追加
def test_web_host_and_static_dir_defaults():
    cfg = load_config(EXAMPLE)
    assert cfg.web.host == "0.0.0.0" and cfg.web.static_dir == ""
```

```python
# tests/test_event_machine.py 追加
def test_min_suffix_metrics_aggregate_min(tmp_path):
    store = EventStore(tmp_path / "e.db")
    m = EventMachine(store, close_hysteresis_s=1.0)
    m.update(100.0, OUT, metrics={"sats_min": 12.0, "corr_gap_s": 3.0})
    m.update(101.0, OUT, metrics={"sats_min": 4.0, "corr_gap_s": 9.0})
    m.update(102.0, OUT, metrics={"sats_min": 8.0, "corr_gap_s": 5.0})
    m.update(103.0, OK); m.update(105.0, OK)
    import json
    peak = json.loads(store.query()[0].peak)
    assert peak["sats_min"] == 4.0          # min, not abs-max
    assert peak["corr_gap_s"] == 9.0        # abs-max unchanged
```

```python
# tests/test_api.py 追加
def test_static_dir_configurable(tmp_path):
    from rtk_monitor.api import create_api
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
```

- [ ] **Step 2: 运行确认失败**
- [ ] **Step 3: 实现** — config.py `WebCfg` 加两字段 + loader 读 `w.get("host","0.0.0.0")`/`w.get("static_dir","")`；example 的 web 段加 `host: 0.0.0.0` 与 `static_dir: ""` 注释行。main.py：uvicorn.Config(..., host=self.cfg.web.host, timeout_graceful_shutdown=5)；两 store 加：

```python
    def checkpoint(self) -> None:
        """Compact the WAL file back into the db (long-running ARM disk hygiene)."""
        self._db.execute("PRAGMA wal_checkpoint(TRUNCATE)")
```

`_cleanup_loop` prune 后调 `self.epochs.checkpoint(); self.events.checkpoint()`（守卫内）。`_diagnosis_tick` metrics 改键 `"sats_min"`。diagnosis/events.py 峰值累计改为：

```python
        if metrics:
            for k, v in metrics.items():
                if k.endswith("_min"):
                    if k not in self._peak or v < self._peak[k]:
                        self._peak[k] = v
                elif abs(v) > abs(self._peak.get(k, 0.0)):
                    self._peak[k] = v
```

api.py：

```python
def _web_dir(app) -> Path:
    cfg = getattr(app, "cfg", None)
    if cfg is not None and getattr(cfg.web, "static_dir", ""):
        return Path(cfg.web.static_dir)
    return Path(__file__).resolve().parents[2] / "web"
```

`create_api` 内 `api.mount("/", StaticFiles(directory=str(_web_dir(app)), html=True), name="web")`。

- [ ] **Step 4: 运行通过 + 全套（154+3）**
- [ ] **Step 5: Commit** — `git commit -m "feat: web host/static-dir config, graceful shutdown, WAL checkpoint, sats_min peak"`

---

### Task 2: vendored 前端库

**Files:**
- Create: `web/vendor/vue.esm-browser.prod.js`、`web/vendor/leaflet/leaflet.js`、`web/vendor/leaflet/leaflet.css`、`web/vendor/leaflet/images/*`、`web/vendor/uplot/uPlot.iife.min.js`、`web/vendor/uplot/uPlot.min.css`、`web/vendor/VENDOR.md`
- Test: `tests/test_web_assets.py`

**Interfaces:**
- Produces: 固定路径的三套库，后续 index.html 按这些路径引用。下载源（pinned）：
  - `https://unpkg.com/vue@3.4.38/dist/vue.esm-browser.prod.js`
  - `https://unpkg.com/leaflet@1.9.4/dist/leaflet.js`、`.../dist/leaflet.css`、`.../dist/images/{marker-icon.png,marker-icon-2x.png,marker-shadow.png,layers.png,layers-2x.png}`
  - `https://unpkg.com/uplot@1.6.30/dist/uPlot.iife.min.js`、`.../dist/uPlot.min.css`
- VENDOR.md 记录名称/版本/URL/License（Vue MIT、Leaflet BSD-2、uPlot MIT）。

- [ ] **Step 1: 写失败测试**

```python
# tests/test_web_assets.py
from pathlib import Path

VENDOR = Path(__file__).resolve().parents[1] / "web" / "vendor"

REQUIRED = [
    "vue.esm-browser.prod.js",
    "leaflet/leaflet.js", "leaflet/leaflet.css", "leaflet/images/marker-icon.png",
    "uplot/uPlot.iife.min.js", "uplot/uPlot.min.css",
    "VENDOR.md",
]


def test_vendor_assets_present_and_nonempty():
    for rel in REQUIRED:
        p = VENDOR / rel
        assert p.is_file(), f"missing {rel}"
        assert p.stat().st_size > 100, f"suspiciously small: {rel}"


def test_vendor_manifest_pins_versions():
    text = (VENDOR / "VENDOR.md").read_text()
    for needle in ("vue@3.4.38", "leaflet@1.9.4", "uplot@1.6.30"):
        assert needle in text
```

- [ ] **Step 2: 运行确认失败**
- [ ] **Step 3: 实现** — `curl -fsSL -o <path> <url>` 逐个下载（校验非空、js 文件头部合理）；写 VENDOR.md（表格：库/版本/URL/License/下载日期）。**若 .gitignore 有任何规则会挡 `web/vendor/`（检查 `git check-ignore`），加豁免 `!web/vendor/**`。**
- [ ] **Step 4: 运行通过 + 全套**
- [ ] **Step 5: Commit** — `git add -f web/vendor tests/test_web_assets.py && git commit -m "feat: vendor Vue 3.4.38, Leaflet 1.9.4, uPlot 1.6.30"`

---

### Task 3: protocol.js 纯逻辑 + node 测试桥

**Files:**
- Create: `web/js/protocol.js`、`tests_js/protocol.test.mjs`
- Test: `tests/test_js.py`（pytest 桥，node 缺席时 skip）

**Interfaces:**
- Produces（后续组件全部消费，名称冻结）：
  - `fixClass(src, q) -> "fixed"|"float"|"bad"|"none"`：src=="rtkrcv" 用 RTKLIB Q（1→fixed，2→float，null/undefined→none，其余→bad）；src=="can"/"gpchc" 用 610 半字节（4→fixed，5→float，null→none，其余→bad）。
  - `badge(status) -> {cls, text}`：status 为 ws 契约的 status 消息；sol 有值按 fixClass("rtkrcv", sol.q) 取 cls，text 为 "RTK 固定"/"浮点解"/"非固定"；sol 为 null 时 can 兜底（fixClass("can", can.q)）；两者皆 null → {cls:"none", text:"无数据"}；verdict.level ∈ warning/serious/critical 时 text 换成 verdict.message（cls 不变，除非 none）。
  - `fmtAge(v)`、`fmtSigma(sdn, sde)`（水平 σ=hypot，米→"1.2 cm"格式）、`fmtT(t)`（本地 HH:MM:SS）、`fmtNum(v, digits, suffix)` —— **null/undefined 一律返回 "—"（corr 空值语义）**。
  - `segmentTrail(points) -> [{cls, latlngs:[[lat,lon],...]}]`：按 fixClass 变化切段（每段 ≥2 点，前段末点复制为后段首点保证连续）。
  - `SPEED_OPTIONS = [1, 10, 60]`。

- [ ] **Step 1: 写失败测试**

```javascript
// tests_js/protocol.test.mjs
import test from "node:test";
import assert from "node:assert/strict";
import { fixClass, badge, fmtAge, fmtSigma, fmtT, fmtNum, segmentTrail, SPEED_OPTIONS }
  from "../web/js/protocol.js";

test("fixClass per source semantics", () => {
  assert.equal(fixClass("rtkrcv", 1), "fixed");
  assert.equal(fixClass("rtkrcv", 2), "float");
  assert.equal(fixClass("rtkrcv", 5), "bad");
  assert.equal(fixClass("rtkrcv", null), "none");
  assert.equal(fixClass("can", 4), "fixed");
  assert.equal(fixClass("can", 5), "float");
  assert.equal(fixClass("can", 3), "bad");
  assert.equal(fixClass("gpchc", undefined), "none");
});

test("badge prefers sol, falls back to can, honors verdict", () => {
  const sol = { q: 1 }, can = { q: 5 };
  assert.deepEqual(badge({ sol, can, verdict: { level: "ok" } }),
                   { cls: "fixed", text: "RTK 固定" });
  assert.deepEqual(badge({ sol: null, can, verdict: { level: "ok" } }),
                   { cls: "float", text: "浮点解" });
  const b = badge({ sol: null, can: null,
                    verdict: { level: "warning", code: "no_data", message: "无数据——检查采集链路" } });
  assert.equal(b.cls, "none");
  assert.equal(b.text, "无数据——检查采集链路");
  const s = badge({ sol, can, verdict: { level: "serious", message: "差分中断 5s" } });
  assert.equal(s.cls, "fixed");
  assert.equal(s.text, "差分中断 5s");
});

test("formatters render null as em-dash", () => {
  assert.equal(fmtAge(null), "—");
  assert.equal(fmtAge(0.8), "0.8 s");
  assert.equal(fmtSigma(0.011, 0.012), "1.6 cm");
  assert.equal(fmtSigma(null, 0.012), "—");
  assert.equal(fmtNum(undefined, 1, " m/s"), "—");
  assert.match(fmtT(0), /^\d{2}:\d{2}:\d{2}$/);
});

test("segmentTrail splits on class change with continuity", () => {
  const pts = [
    { lat: 1, lon: 1, src: "can", q: 4 },
    { lat: 2, lon: 2, src: "can", q: 4 },
    { lat: 3, lon: 3, src: "can", q: 5 },
    { lat: 4, lon: 4, src: "can", q: 5 },
  ];
  const segs = segmentTrail(pts);
  assert.equal(segs.length, 2);
  assert.equal(segs[0].cls, "fixed");
  assert.equal(segs[1].cls, "float");
  assert.deepEqual(segs[0].latlngs.at(-1), [2, 2]);
  assert.deepEqual(segs[1].latlngs[0], [2, 2]);   // continuity point
  assert.equal(segs[1].latlngs.length, 3);
});

test("speed options", () => assert.deepEqual(SPEED_OPTIONS, [1, 10, 60]));
```

- [ ] **Step 2: 运行确认失败** — `node --test tests_js/`，FAIL（模块不存在）
- [ ] **Step 3: 实现 protocol.js**

```javascript
// web/js/protocol.js — pure logic, no DOM; tested with node --test.
export const SPEED_OPTIONS = [1, 10, 60];

export function fixClass(src, q) {
  if (q === null || q === undefined) return "none";
  if (src === "rtkrcv") return q === 1 ? "fixed" : q === 2 ? "float" : "bad";
  return q === 4 ? "fixed" : q === 5 ? "float" : "bad";   // 610 nibble (can/gpchc)
}

const _BADGE_TEXT = { fixed: "RTK 固定", float: "浮点解", bad: "非固定", none: "无数据" };

export function badge(status) {
  const sol = status.sol, can = status.can, v = status.verdict || {};
  let cls;
  if (sol && sol.q !== null && sol.q !== undefined) cls = fixClass("rtkrcv", sol.q);
  else if (can && can.q !== null && can.q !== undefined) cls = fixClass("can", can.q);
  else cls = "none";
  let text = _BADGE_TEXT[cls];
  if (["warning", "serious", "critical"].includes(v.level) && v.message) text = v.message;
  return { cls, text };
}

export function fmtNum(v, digits, suffix) {
  if (v === null || v === undefined || Number.isNaN(v)) return "—";
  return v.toFixed(digits) + (suffix || "");
}

export const fmtAge = (v) => fmtNum(v, 1, " s");

export function fmtSigma(sdn, sde) {
  if (sdn === null || sdn === undefined || sde === null || sde === undefined) return "—";
  return (Math.hypot(sdn, sde) * 100).toFixed(1) + " cm";
}

export function fmtT(t) {
  if (t === null || t === undefined) return "—";
  return new Date(t * 1000).toTimeString().slice(0, 8);
}

export function segmentTrail(points) {
  const segs = [];
  let cur = null;
  for (const p of points) {
    const cls = fixClass(p.src, p.q);
    if (!cur || cur.cls !== cls) {
      const start = cur ? [cur.latlngs.at(-1)] : [];
      cur = { cls, latlngs: [...start] };
      segs.push(cur);
    }
    cur.latlngs.push([p.lat, p.lon]);
  }
  return segs.filter((s) => s.latlngs.length >= 2);
}
```

- [ ] **Step 4: node 测试通过；写 pytest 桥并通过**

```python
# tests/test_js.py
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.skipif(shutil.which("node") is None, reason="node not available")
def test_js_protocol_suite():
    r = subprocess.run(["node", "--test", "tests_js/"], cwd=ROOT,
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stdout + "\n" + r.stderr
```

- [ ] **Step 5: Commit** — `git commit -m "feat: frontend protocol pure logic with node tests"`

---

### Task 4: 页面骨架（index.html / style.css / store / ws / app）

**Files:**
- Create: `web/index.html`（覆盖占位页）、`web/style.css`、`web/js/store.js`、`web/js/ws.js`、`web/app.js`
- Test: `tests/test_web_assets.py`（追加引用完整性测试）

**Interfaces:**
- Produces: `store.js` 导出 `createStore(Vue)` → reactive 对象：`{connected:false, replaying:false, status:null, trails:{can:[],rtkrcv:[],gpchc:[]}, events:[], series:{t:[],sats:[],age:[],sigma:[],ratio:[]}, lastError:null}` 与方法 `applyMessage(msg)`（按契约分发：status→status+trails(sol/gpchc 点)+series 追加（滚动 1800 点）；position→trails.can 追加（每 src 上限 3600 点，超出裁头）；event→events 头插（上限 200）+ replaying 状态由 replay_end/error 维护）。`ws.js` 导出 `class WsClient { constructor(url, onMessage, onState) ; connect(); sendReplay(t0,t1,speed); sendLive(); close(); }`（自动重连 1→5s 退避，onState(connected:boolean)）。`app.js`：创建 Vue app，全局 provide store 与 ws。index.html：CSS Grid 布局四区 + 各组件挂载点 + `<script type="module" src="/app.js">`。
- 组件文件（Task 5-7 实现）在 index.html 中占位为 `<div id="statusbar">`、`<div id="map">`、`<div id="skyplot-box">`、`<div id="eventlist">`、`<div id="timelines">`、`<div id="replaybar">`。

- [ ] **Step 1: 写失败测试（追加）**

```python
# tests/test_web_assets.py 追加
import re

WEB = Path(__file__).resolve().parents[1] / "web"


def test_index_references_resolve():
    html = (WEB / "index.html").read_text()
    refs = re.findall(r'(?:src|href)="/([^"]+)"', html)
    assert refs, "index.html should reference local assets"
    for ref in refs:
        assert (WEB / ref).is_file(), f"broken reference: /{ref}"


def test_index_is_dark_chinese_ui():
    html = (WEB / "index.html").read_text()
    assert 'lang="zh-CN"' in html and "rtk-monitor" in html
    css = (WEB / "style.css").read_text()
    assert "--bg" in css                      # theme variables present
```

- [ ] **Step 2: 运行确认失败**（占位 index 无引用）
- [ ] **Step 3: 实现**

index.html：

```html
<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>rtk-monitor</title>
<link rel="stylesheet" href="/vendor/leaflet/leaflet.css">
<link rel="stylesheet" href="/vendor/uplot/uPlot.min.css">
<link rel="stylesheet" href="/style.css">
</head>
<body>
<div id="banner" class="banner hidden">数据中断，重连中…</div>
<header id="statusbar"></header>
<main>
  <div id="map"></div>
  <aside>
    <section id="skyplot-box"><h3>天空图</h3><canvas id="skyplot" width="300" height="300"></canvas></section>
    <section id="eventlist-box"><h3>事件</h3><div id="eventlist"></div></section>
  </aside>
</main>
<footer>
  <div id="timelines">
    <div class="chart" id="chart-sats"></div><div class="chart" id="chart-age"></div>
    <div class="chart" id="chart-sigma"></div><div class="chart" id="chart-ratio"></div>
  </div>
  <div id="replaybar"></div>
</footer>
<script src="/vendor/leaflet/leaflet.js"></script>
<script src="/vendor/uplot/uPlot.iife.min.js"></script>
<script type="module" src="/app.js"></script>
</body>
</html>
```

style.css（要点，完整写出）：`:root{--bg:#101418;--panel:#1a2027;--ink:#d8dee6;--dim:#8b95a3;--fixed:#3fb96c;--float:#e0b23c;--bad:#e05c4f;--none:#5a6472}`；body 满屏 grid：`grid-template-rows:auto 1fr auto`；main 内 `display:grid;grid-template-columns:1fr 320px`；badge 类 `.badge.fixed{background:var(--fixed)}` 等四态；`.banner{position:fixed;top:0;...;background:var(--bad)} .hidden{display:none}`；事件级别色条；图表容器高 110px。

store.js：

```javascript
// web/js/store.js
import { fixClass } from "/js/protocol.js";

const TRAIL_MAX = 3600, SERIES_MAX = 1800, EVENTS_MAX = 200;

export function createStore(Vue) {
  const s = Vue.reactive({
    connected: false, replaying: false, status: null, lastError: null,
    trails: { can: [], rtkrcv: [], gpchc: [] },
    events: [],
    series: { t: [], sats: [], age: [], sigma: [], ratio: [] },
  });

  function pushTrail(src, lat, lon, q) {
    if (lat === null || lat === undefined) return;
    const arr = s.trails[src];
    arr.push({ lat, lon, src, q });
    if (arr.length > TRAIL_MAX) arr.splice(0, arr.length - TRAIL_MAX);
  }

  s.applyMessage = (m) => {
    if (m.type === "status") {
      s.status = m;
      if (m.sol) pushTrail("rtkrcv", m.sol.lat, m.sol.lon, m.sol.q);
      if (m.gpchc) pushTrail("gpchc", m.gpchc.lat, m.gpchc.lon, m.gpchc.q);
      const sol = m.sol || {}, can = m.can || {};
      for (const [k, v] of Object.entries({
        t: m.t, sats: sol.sats ?? can.sats ?? null,
        age: sol.age ?? can.age ?? null,
        sigma: (sol.sdn != null && sol.sde != null) ? Math.hypot(sol.sdn, sol.sde) : null,
        ratio: sol.ratio ?? null,
      })) {
        s.series[k].push(v);
        if (s.series[k].length > SERIES_MAX) s.series[k].shift();
      }
    } else if (m.type === "position") {
      pushTrail(m.src, m.lat, m.lon, m.q);
    } else if (m.type === "event") {
      s.events.unshift({ action: m.action, ...m.event });
      if (s.events.length > EVENTS_MAX) s.events.pop();
    } else if (m.type === "replay_end") {
      s.replaying = false;
    } else if (m.type === "error") {
      s.lastError = m.detail; s.replaying = false;
    }
  };

  s.clearForReplay = () => {
    s.trails = { can: [], rtkrcv: [], gpchc: [] };
    s.series = { t: [], sats: [], age: [], sigma: [], ratio: [] };
    s.replaying = true;
  };
  return s;
}
```

ws.js：

```javascript
// web/js/ws.js
export class WsClient {
  constructor(url, onMessage, onState) {
    this.url = url; this.onMessage = onMessage; this.onState = onState;
    this.backoff = 1000; this.ws = null; this.closed = false;
  }
  connect() {
    this.ws = new WebSocket(this.url);
    this.ws.onopen = () => { this.backoff = 1000; this.onState(true); };
    this.ws.onmessage = (ev) => this.onMessage(JSON.parse(ev.data));
    this.ws.onclose = () => {
      this.onState(false);
      if (this.closed) return;
      setTimeout(() => this.connect(), this.backoff);
      this.backoff = Math.min(this.backoff * 2, 5000);
    };
    this.ws.onerror = () => this.ws.close();
  }
  _send(obj) { if (this.ws && this.ws.readyState === 1) this.ws.send(JSON.stringify(obj)); }
  sendReplay(t0, t1, speed) { this._send({ cmd: "replay", t0, t1, speed }); }
  sendLive() { this._send({ cmd: "live" }); }
  close() { this.closed = true; if (this.ws) this.ws.close(); }
}
```

app.js：

```javascript
// web/app.js
import * as Vue from "/vendor/vue.esm-browser.prod.js";
import { createStore } from "/js/store.js";
import { WsClient } from "/js/ws.js";
import { mountStatusbar } from "/js/statusbar.js";
import { mountEventlist } from "/js/eventlist.js";
import { mountReplaybar } from "/js/replaybar.js";
import { MapView } from "/js/mapview.js";
import { Timelines } from "/js/timeline.js";
import { Skyplot } from "/js/skyplot.js";

const store = createStore(Vue);
const proto = location.protocol === "https:" ? "wss" : "ws";
const ws = new WsClient(`${proto}://${location.host}/ws`, store.applyMessage,
                        (up) => { store.connected = up;
                                  document.getElementById("banner").classList.toggle("hidden", up); });
ws.connect();

mountStatusbar(Vue, store);
mountEventlist(Vue, store, ws);
mountReplaybar(Vue, store, ws);
const map = new MapView("map", store);
const tl = new Timelines(store);
const sky = new Skyplot("skyplot", store);
setInterval(() => { map.render(); tl.render(); sky.render(); }, 1000);
```

（组件文件 Task 5-7 创建；本任务先建立四个空导出文件避免 import 失败：每个文件导出对应的 no-op 函数/类，后续任务替换——文件必须存在且语法有效，`test_index_references_resolve` 只查 html 引用，module import 链由 Task 10 的整页 smoke 验证。）

- [ ] **Step 4: 运行通过 + 全套**
- [ ] **Step 5: Commit** — `git commit -m "feat: SPA skeleton — layout, dark theme, store, ws client"`

---

### Task 5: 状态条 + 事件列表 + 回放条（Vue 组件）

**Files:**
- Create（替换 no-op）: `web/js/statusbar.js`、`web/js/eventlist.js`、`web/js/replaybar.js`
- Test: `tests_js/components.test.mjs`（纯逻辑部分）+ `tests/test_js.py` 已有桥自动纳入

**Interfaces:**
- Consumes: store 结构（Task 4）、protocol.js 全部导出、WsClient.sendReplay/sendLive。
- Produces: `mountStatusbar(Vue, store)`、`mountEventlist(Vue, store, ws)`（点击事件行 → `ws` 触发回放：`t0 = ev.t - 30`，`t1 = (ev.t_close ?? ev.t + 60) + 30`，speed 10，且 `store.clearForReplay()`）、`mountReplaybar(Vue, store, ws)`（datetime-local ×2 + speed select(SPEED_OPTIONS) + 开始回放/回到实时按钮；replaying 时显示"回放中…"横幅样式；开始前 `store.clearForReplay()`；回到实时 `ws.sendLive()` 并置 `store.replaying=false`）。另导出纯函数 `eventReplayWindow(ev) -> {t0,t1}`（上述规则，node 可测）放 eventlist.js。

- [ ] **Step 1: 写失败测试**

```javascript
// tests_js/components.test.mjs
import test from "node:test";
import assert from "node:assert/strict";
import { eventReplayWindow } from "../web/js/eventlist.js";

test("event replay window: closed event", () => {
  const w = eventReplayWindow({ t: 100, t_close: 160 });
  assert.deepEqual(w, { t0: 70, t1: 190 });
});

test("event replay window: open event uses t+60 fallback", () => {
  const w = eventReplayWindow({ t: 100, t_close: null });
  assert.deepEqual(w, { t0: 70, t1: 190 });
});
```

- [ ] **Step 2: 运行确认失败** — `node --test tests_js/`
- [ ] **Step 3: 实现三组件**

statusbar.js：

```javascript
// web/js/statusbar.js
import { badge, fmtAge, fmtSigma, fmtNum, fmtT } from "/js/protocol.js";

export function mountStatusbar(Vue, store) {
  Vue.createApp({
    computed: {
      st() { return store.status || {}; },
      b() { return this.st.verdict ? badge(this.st) : { cls: "none", text: "无数据" }; },
      sol() { return this.st.sol || {}; },
      can() { return this.st.can || {}; },
      speed() { return fmtNum(this.can.speed, 1, " m/s"); },
      sats() { return this.sol.sats ?? this.can.sats ?? "—"; },
      age() { return fmtAge(this.sol.age ?? this.can.age ?? null); },
      sigma() { return fmtSigma(this.sol.sdn, this.sol.sde); },
      clock() { return fmtT(this.st.t); },
      replaying() { return store.replaying; },
    },
    template: `
      <div class="statusbar">
        <span :class="'badge ' + b.cls">{{ b.text }}</span>
        <span v-if="replaying" class="tag replay">回放</span>
        <span class="kv">卫星 <b>{{ sats }}</b></span>
        <span class="kv">龄期 <b>{{ age }}</b></span>
        <span class="kv">σ <b>{{ sigma }}</b></span>
        <span class="kv">速度 <b>{{ speed }}</b></span>
        <span class="kv time">{{ clock }}</span>
      </div>`,
  }).mount("#statusbar");
}
```

eventlist.js：

```javascript
// web/js/eventlist.js
import { fmtT } from "/js/protocol.js";

export function eventReplayWindow(ev) {
  const end = (ev.t_close ?? ev.t + 60);
  return { t0: ev.t - 30, t1: end + 30 };
}

export function mountEventlist(Vue, store, ws) {
  Vue.createApp({
    computed: { events() { return store.events; } },
    methods: {
      fmtT,
      jump(ev) {
        const { t0, t1 } = eventReplayWindow(ev);
        store.clearForReplay();
        ws.sendReplay(t0, t1, 10);
      },
    },
    template: `
      <ul class="events">
        <li v-for="e in events" :class="'lv-' + (e.level || 'info')" @click="jump(e)">
          <span class="et">{{ fmtT(e.t) }}</span>
          <span class="ea">{{ e.action === 'open' ? '▲' : '▼' }}</span>
          <span class="em">{{ e.message }}</span>
        </li>
        <li v-if="!events.length" class="empty">暂无事件</li>
      </ul>`,
  }).mount("#eventlist");
}
```

replaybar.js：

```javascript
// web/js/replaybar.js
import { SPEED_OPTIONS } from "/js/protocol.js";

export function mountReplaybar(Vue, store, ws) {
  Vue.createApp({
    data() {
      const now = new Date(), ago = new Date(Date.now() - 3600e3);
      const fmt = (d) => new Date(d - d.getTimezoneOffset() * 60e3).toISOString().slice(0, 16);
      return { t0: fmt(ago), t1: fmt(now), speed: 10, opts: SPEED_OPTIONS };
    },
    computed: { replaying() { return store.replaying; },
                error() { return store.lastError; } },
    methods: {
      start() {
        const a = new Date(this.t0).getTime() / 1000, b = new Date(this.t1).getTime() / 1000;
        if (!(b > a)) return;
        store.lastError = null;
        store.clearForReplay();
        ws.sendReplay(a, b, Number(this.speed));
      },
      live() { ws.sendLive(); store.replaying = false; },
    },
    template: `
      <div class="replaybar">
        <label>回放 <input type="datetime-local" v-model="t0"></label>
        <label>至 <input type="datetime-local" v-model="t1"></label>
        <select v-model="speed"><option v-for="s in opts" :value="s">{{ s }}×</option></select>
        <button @click="start" :disabled="replaying">开始回放</button>
        <button @click="live" :disabled="!replaying">回到实时</button>
        <span v-if="error" class="err">{{ error }}</span>
      </div>`,
  }).mount("#replaybar");
}
```

- [ ] **Step 4: node 测试通过 + pytest 全套（test_js 桥连带跑新测试）**
- [ ] **Step 5: Commit** — `git commit -m "feat: statusbar, event list with replay jump, replay controls"`

---

### Task 6: 地图（瓦片/网格退化/三轨迹/位置箭头/σ圈）

**Files:**
- Create（替换 no-op）: `web/js/mapview.js`
- Test: `tests_js/mapview.test.mjs`（纯逻辑：脏检查/裁剪决策抽为可测函数）

**Interfaces:**
- Consumes: store.trails/status；全局 `L`（Leaflet，非 module script 已加载）；protocol.segmentTrail。
- Produces: `class MapView { constructor(elId, store); render(); }`。行为：构造时探测 `fetch('/tiles/12/3000/1500.png')`——2xx 用 `/tiles/{z}/{x}/{y}.png` tileLayer（maxZoom 22），否则安装网格退化层（L.GridLayer 子类，canvas 画边框+z/x/y 文本）；render() 每秒：三轨迹按 `segmentTrail` 重画（每 src 一个 L.layerGroup，先 clearLayers；颜色 CSS 变量映射 `{fixed:'#3fb96c',float:'#e0b23c',bad:'#e05c4f',none:'#5a6472'}`；rtkrcv 实线 weight3、can 实线 weight2 透明度 .8、gpchc 虚线 dashArray '4 6'）；当前位置取 status.can（兜底 sol）画旋转箭头 divIcon（CSS transform rotate(heading)deg）+ σ 水平圈（L.circle 半径 = hypot(sdn,sde) 米，sol 缺失不画）；首个定位点时 setView zoom 17，此后不抢用户视角（仅当用户未拖动过——`_userMoved` 标志由 map 'dragstart' 置位）。导出纯函数 `trailColor(cls) -> hex`（node 测试）。

- [ ] **Step 1: 写失败测试**

```javascript
// tests_js/mapview.test.mjs
import test from "node:test";
import assert from "node:assert/strict";
import { trailColor } from "../web/js/mapview.js";

test("trail colors map fix classes", () => {
  assert.equal(trailColor("fixed"), "#3fb96c");
  assert.equal(trailColor("float"), "#e0b23c");
  assert.equal(trailColor("bad"), "#e05c4f");
  assert.equal(trailColor("none"), "#5a6472");
});
```

- [ ] **Step 2: 运行确认失败**
- [ ] **Step 3: 实现 mapview.js**（完整实现按 Interfaces 展开；关键骨架：）

```javascript
// web/js/mapview.js
import { segmentTrail } from "/js/protocol.js";

const COLORS = { fixed: "#3fb96c", float: "#e0b23c", bad: "#e05c4f", none: "#5a6472" };
export const trailColor = (cls) => COLORS[cls];

const GridFallback = typeof L !== "undefined" ? L.GridLayer.extend({
  createTile(coords) {
    const c = document.createElement("canvas");
    c.width = c.height = 256;
    const g = c.getContext("2d");
    g.strokeStyle = "#2a3340"; g.strokeRect(0, 0, 256, 256);
    g.fillStyle = "#5a6472"; g.font = "12px sans-serif";
    g.fillText(`${coords.z}/${coords.x}/${coords.y}`, 8, 16);
    return c;
  },
}) : null;

export class MapView {
  constructor(elId, store) {
    this.store = store;
    this.map = L.map(elId, { zoomControl: true }).setView([0, 0], 3);
    this.map.on("dragstart", () => { this._userMoved = true; });
    fetch("/tiles/12/3000/1500.png").then((r) => {
      if (r.ok) L.tileLayer("/tiles/{z}/{x}/{y}.png", { maxZoom: 22 }).addTo(this.map);
      else new GridFallback().addTo(this.map);
    }).catch(() => new GridFallback().addTo(this.map));
    this.groups = { can: L.layerGroup().addTo(this.map),
                    rtkrcv: L.layerGroup().addTo(this.map),
                    gpchc: L.layerGroup().addTo(this.map) };
    this.marker = null; this.sigma = null; this._centered = false;
  }

  _style(src, cls) {
    const base = { color: trailColor(cls) };
    if (src === "rtkrcv") return { ...base, weight: 3 };
    if (src === "can") return { ...base, weight: 2, opacity: 0.8 };
    return { ...base, weight: 2, dashArray: "4 6" };
  }

  render() {
    for (const src of ["can", "rtkrcv", "gpchc"]) {
      const g = this.groups[src];
      g.clearLayers();
      for (const seg of segmentTrail(this.store.trails[src]))
        L.polyline(seg.latlngs, this._style(src, seg.cls)).addTo(g);
    }
    const st = this.store.status || {};
    const pos = st.can && st.can.lat != null ? st.can : (st.sol && st.sol.lat != null ? st.sol : null);
    if (!pos) return;
    const ll = [pos.lat, pos.lon];
    const heading = pos.heading ?? 0;
    const icon = L.divIcon({ className: "veh",
      html: `<div class="veh-arrow" style="transform:rotate(${heading}deg)">▲</div>`,
      iconSize: [24, 24], iconAnchor: [12, 12] });
    if (!this.marker) this.marker = L.marker(ll, { icon }).addTo(this.map);
    else { this.marker.setLatLng(ll); this.marker.setIcon(icon); }
    const sol = st.sol;
    if (sol && sol.sdn != null && sol.sde != null) {
      const r = Math.hypot(sol.sdn, sol.sde);
      if (!this.sigma) this.sigma = L.circle(ll, { radius: r, color: "#4a90d9", weight: 1 }).addTo(this.map);
      else { this.sigma.setLatLng(ll); this.sigma.setRadius(r); }
    }
    if (!this._centered) { this.map.setView(ll, 17); this._centered = true; }
    else if (!this._userMoved) this.map.panTo(ll, { animate: false });
  }
}
```

（style.css 追加 `.veh-arrow{color:#4a90d9;font-size:20px;text-shadow:0 0 3px #000}`。注意文件顶部引用全局 `L`——mapview.js 仍是 ES module，由 app.js import；`GridFallback` 定义做了 typeof 守卫使 node --test 能 import 本文件取 trailColor。）

- [ ] **Step 4: node + pytest 全套通过**
- [ ] **Step 5: Commit** — `git commit -m "feat: map view — offline tiles with grid fallback, state-colored trails, vehicle marker"`

---

### Task 7: 时间线 ×4 + 天空图

**Files:**
- Create（替换 no-op）: `web/js/timeline.js`、`web/js/skyplot.js`
- Test: `tests_js/timeline.test.mjs`

**Interfaces:**
- Consumes: store.series（t/sats/age/sigma/ratio，等长数组）、全局 `uPlot`（iife 已加载）、store.status.sol.sats_json（天空图数据，当前恒 null——占位态）。
- Produces: `class Timelines { constructor(store); render(); }`——四张 uPlot 图（卫星数/龄期 s/σ 水平 cm/ratio），挂 `#chart-sats` 等四容器，同 `cursor.sync` key "rtk"，深色轴样式，series 数据直接引用 store.series（render() 调 `setData`）；σ 系列显示为 cm（导出纯函数 `sigmaSeriesCm(sigmaArr)`：米→cm、null 保留）。`class Skyplot { constructor(canvasId, store); render(); }`——sol.sats_json 为 null/undefined 时画占位（同心圆 30/60° + "等待 $SAT 数据" 文本）；有值时（JSON 数组 [{sat,az,el,snr,used}]）按方位角/高度角投影画点，SNR 着色（>40 绿 / 35-40 黄 / <35 红），未用卫星空心。

- [ ] **Step 1: 写失败测试**

```javascript
// tests_js/timeline.test.mjs
import test from "node:test";
import assert from "node:assert/strict";
import { sigmaSeriesCm } from "../web/js/timeline.js";
import { skyXY } from "../web/js/skyplot.js";

test("sigma meters to cm, nulls preserved", () => {
  assert.deepEqual(sigmaSeriesCm([0.011, null, 0.02]), [1.1, null, 2]);
});

test("sky projection: el=90 center, el=0 rim, az=90 east", () => {
  const c = skyXY(0, 90, 100);      // az irrelevant at zenith
  assert.ok(Math.abs(c.x - 100) < 1e-9 && Math.abs(c.y - 100) < 1e-9);
  const e = skyXY(90, 0, 100);      // due east on the rim
  assert.ok(Math.abs(e.x - 200) < 1e-9 && Math.abs(e.y - 100) < 1e-9);
  const n = skyXY(0, 0, 100);       // due north on the rim (up)
  assert.ok(Math.abs(n.x - 100) < 1e-9 && Math.abs(n.y - 0) < 1e-9);
});
```

- [ ] **Step 2: 运行确认失败**
- [ ] **Step 3: 实现**

timeline.js（骨架，完整展开）：

```javascript
// web/js/timeline.js
export function sigmaSeriesCm(arr) {
  return arr.map((v) => (v === null || v === undefined ? null : Math.round(v * 1000) / 10));
}

const AXIS = { stroke: "#8b95a3", grid: { stroke: "#232b35" }, ticks: { stroke: "#232b35" } };

function mkChart(el, label, color) {
  return new uPlot({
    width: el.clientWidth || 600, height: 110,
    cursor: { sync: { key: "rtk" } },
    legend: { show: false },
    scales: { x: { time: true } },
    axes: [ { ...AXIS }, { ...AXIS, label } ],
    series: [ {}, { label, stroke: color, width: 2, spanGaps: false } ],
  }, [[], []], el);
}

export class Timelines {
  constructor(store) {
    this.store = store;
    this.charts = {
      sats: mkChart(document.getElementById("chart-sats"), "卫星数", "#4a90d9"),
      age: mkChart(document.getElementById("chart-age"), "龄期 s", "#3fb96c"),
      sigma: mkChart(document.getElementById("chart-sigma"), "σ cm", "#e0b23c"),
      ratio: mkChart(document.getElementById("chart-ratio"), "ratio", "#b07ad9"),
    };
  }
  render() {
    const s = this.store.series;
    this.charts.sats.setData([s.t, s.sats]);
    this.charts.age.setData([s.t, s.age]);
    this.charts.sigma.setData([s.t, sigmaSeriesCm(s.sigma)]);
    this.charts.ratio.setData([s.t, s.ratio]);
  }
}
```

skyplot.js：

```javascript
// web/js/skyplot.js
export function skyXY(azDeg, elDeg, r) {
  const rho = r * (90 - elDeg) / 90;
  const a = (azDeg - 90) * Math.PI / 180;   // az 0 = north = up
  return { x: r + rho * Math.cos(a), y: r + rho * Math.sin(a) };
}

export class Skyplot {
  constructor(canvasId, store) {
    this.el = document.getElementById(canvasId);
    this.store = store;
  }
  render() {
    const g = this.el.getContext("2d"), W = this.el.width, r = W / 2;
    g.clearRect(0, 0, W, W);
    g.strokeStyle = "#2a3340";
    for (const f of [1, 2 / 3, 1 / 3]) {
      g.beginPath(); g.arc(r, r, r * f - 1, 0, 7); g.stroke();
    }
    const sol = (this.store.status || {}).sol;
    let sats = null;
    try { sats = sol && sol.sats_json ? JSON.parse(sol.sats_json) : null; } catch { /* keep null */ }
    if (!sats || !sats.length) {
      g.fillStyle = "#5a6472"; g.font = "13px sans-serif"; g.textAlign = "center";
      g.fillText("等待 $SAT 数据", r, r - 6);
      g.fillText("（真机接通 stat 流后显示）", r, r + 12);
      return;
    }
    for (const s of sats) {
      const { x, y } = skyXY(s.az, s.el, r);
      g.beginPath(); g.arc(x, y, 5, 0, 7);
      g.fillStyle = s.snr > 40 ? "#3fb96c" : s.snr >= 35 ? "#e0b23c" : "#e05c4f";
      if (s.used) g.fill(); else { g.strokeStyle = g.fillStyle; g.stroke(); }
      g.fillStyle = "#8b95a3"; g.font = "9px sans-serif"; g.fillText(s.sat, x + 6, y + 3);
    }
  }
}
```

- [ ] **Step 4: node + pytest 全套通过**
- [ ] **Step 5: Commit** — `git commit -m "feat: uPlot timelines and skyplot with waiting-for-stat placeholder"`

---

### Task 8: 部署与瓦片文档

**Files:**
- Create: `docs/deploy.md`、`docs/tiles-howto.md`
- Modify: `docs/integration-rtkrcv.md`（追加 UI 联调项）、`README.md`（简介 + 指向文档）

**Interfaces:** 文档任务，无代码接口。内容要求（中文，全部写完整，不留 TBD）：
- `docs/deploy.md`：①硬件/网络前提（车载 ARM、can0、610 与平台可达）；②610 网页配置步骤（Server7 勾板卡原始、Client7 勾卫导+IMU、CAN 保持、差分接入方式确认——引 spec §1.2）；③config.yaml 全字段说明表（含 web.host/static_dir/tiles_path、db_retention_days、诊断阈值一览）；④裸机运行与 Docker 运行两种方式；⑤安全 posture（引 docs/ws-contract.md：信任车载 LAN、无鉴权、web.host 可收紧）；⑥常见问题（rtkrcv 不固定→查 integration 清单；界面无数据→查 WS/采集事件表）。
- `docs/tiles-howto.md`：矿区影像 → MBTiles 三条路线：A. 已有 GeoTIFF：`gdal_translate -of MBTILES ortho.tif mine.mbtiles && gdaladdo mine.mbtiles 2 4 8 16`；B. QGIS 导出 XYZ tiles → `mb-util --image_format=png tiles/ mine.mbtiles`；C. SAS.Planet 下载卫星影像导出 MBTiles。坐标必须 EPSG:3857；放置路径与 config `web.tiles_path` 对应；无瓦片时界面自动网格退化（说明这是预期行为）。
- `docs/integration-rtkrcv.md` 追加：UI 项（浏览器打开 8080：状态条变绿、轨迹三色、事件点击回放、报告页打印）+ 48h DB 观察项（已有）+ 「kill rtkrcv 进程 → 60s 内状态条显示 no_solution」演练。
- README.md：项目一句话 + 快速开始（config 拷贝、运行命令、浏览器地址）+ 文档索引（deploy/tiles/integration/ws-contract/specs）。

- [ ] **Step 1-4: 写四个文档；`pytest -q` 确认不破（文档无测试）**
- [ ] **Step 5: Commit** — `git commit -m "docs: deploy guide, tiles howto, UI integration checklist, README"`

---

### Task 9: Docker 交付

**Files:**
- Create: `Dockerfile`、`docker-compose.yml`、`.dockerignore`
- Test: `tests/test_docker.py`（docker 存在时构建冒烟；否则 skip）

**Interfaces:**
- Produces: 双阶段 Dockerfile（阶段1 编 demo5 rtkrcv；阶段2 python:3.11-slim 运行层，含 web/ 与 config 示例，rtkrcv 落 /usr/local/bin）；compose：host 网络、restart always、/data 卷、config 从 /data/config.yaml 读。容器内 config 必须设 `web.static_dir: /app/web`（pip install 后源码相对路径失效——Task 1 已做成可配置；compose 示例与 deploy.md 均要写明）。

- [ ] **Step 1: 写文件**

```dockerfile
# Dockerfile
FROM python:3.11-slim AS rtkbuild
RUN apt-get update && apt-get install -y --no-install-recommends git build-essential \
    && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 --branch demo5 https://github.com/rtklibexplorer/RTKLIB.git /rtklib \
    && make -C /rtklib/app/consapp/rtkrcv/gcc -j"$(nproc)"

FROM python:3.11-slim
WORKDIR /app
COPY pyproject.toml README.md ./
COPY src ./src
RUN pip install --no-cache-dir .
COPY web ./web
COPY config.yaml.example ./config.yaml.example
COPY --from=rtkbuild /rtklib/app/consapp/rtkrcv/gcc/rtkrcv /usr/local/bin/rtkrcv
CMD ["python", "-m", "rtk_monitor.main", "/data/config.yaml"]
```

```yaml
# docker-compose.yml
services:
  rtk-monitor:
    build: .
    network_mode: host        # SocketCAN + LAN services
    restart: always
    volumes:
      - /data:/data           # config.yaml, gnsslog, rtk.db, mbtiles all live here
```

`.dockerignore`：`.git`、`.venv`、`.claude`、`.superpowers`、`tests`、`tests_js`、`third_party`、`tools/bin`、`**/__pycache__`、`docs/superpowers`。

- [ ] **Step 2: 写构建冒烟测试**

```python
# tests/test_docker.py
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_dockerfile_has_two_stages():
    text = (ROOT / "Dockerfile").read_text()
    assert text.count("FROM ") == 2 and "rtkbuild" in text
    assert "web ./web" in text and "/usr/local/bin/rtkrcv" in text


@pytest.mark.skipif(shutil.which("docker") is None, reason="docker not available")
@pytest.mark.slow
def test_docker_image_builds():
    r = subprocess.run(["docker", "build", "-q", "."], cwd=ROOT,
                       capture_output=True, text=True, timeout=900)
    assert r.returncode == 0, r.stderr[-2000:]
```

（pyproject 的 pytest 配置加 `markers = ["slow: long-running (docker build)"]`；日常 `pytest -q` 全跑，构建 ~2-5 分钟属可接受一次性验证——实现者必须真跑一次并在报告贴输出。）

- [ ] **Step 3: 真跑 `pytest tests/test_docker.py -v`（含构建）确认通过**
- [ ] **Step 4: 全套通过**
- [ ] **Step 5: Commit** — `git commit -m "feat: docker multi-stage delivery with rtkrcv build"`

---

### Task 10: 整页集成冒烟 + 收尾

**Files:**
- Test: `tests/test_web_smoke.py`
- Modify: `tests/test_web_assets.py`（若 index 引用在 Task 5-7 中新增，确保仍全绿——无代码则免改）

**Interfaces:** 消费全装配。验证：静态链全通、ES module 引用图完整（不起浏览器：解析 js import 图逐个 GET）、node --check 全部 js 语法、WS live 冒烟已有（Plan 3a e2e）。

- [ ] **Step 1: 写测试**

```python
# tests/test_web_smoke.py
import re
import shutil
import subprocess
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

WEB = Path(__file__).resolve().parents[1] / "web"


def _client(tmp_path):
    from rtk_monitor.api import create_api
    from tests.test_api import _FakeApp
    return TestClient(create_api(_FakeApp(tmp_path)))


def _local_imports(js_text):
    return re.findall(r'from\s+"(/[^"]+)"', js_text) + \
           re.findall(r'import\s+"(/[^"]+)"', js_text)


def test_module_graph_resolves_over_http(tmp_path):
    c = _client(tmp_path)
    seen, todo = set(), ["/app.js"]
    while todo:
        path = todo.pop()
        if path in seen:
            continue
        seen.add(path)
        r = c.get(path)
        assert r.status_code == 200, f"unresolved module {path}"
        if path.endswith(".js"):
            todo.extend(_local_imports(r.text))
    assert "/js/protocol.js" in seen and "/js/mapview.js" in seen


@pytest.mark.skipif(shutil.which("node") is None, reason="node not available")
def test_all_js_files_parse():
    for p in sorted(WEB.rglob("*.js")):
        if "vendor" in p.parts:
            continue
        r = subprocess.run(["node", "--check", str(p)], capture_output=True, text=True)
        assert r.returncode == 0, f"{p}: {r.stderr}"
```

- [ ] **Step 2: 运行确认（应直接绿；若红修复引用）**
- [ ] **Step 3: 手工验证一次整页**：`.venv/bin/python -m rtk_monitor.main <临时config web port 8081>` + `tools/replay_sources.py` 喂数（或 fake rtkrcv 配置），`curl http://127.0.0.1:8081/` 与 `/app.js` 抽查 200；报告记录命令与输出（无浏览器截图要求）。
- [ ] **Step 4: 全套 pytest 通过（含 slow）**
- [ ] **Step 5: Commit** — `git commit -m "test: full-page module-graph smoke and js syntax gate"`

---

## Self-Review 记录

- **Spec 覆盖**：§5 全部界面元素（状态条 T5、地图三轨迹/箭头/σ圈 T6、天空图+事件列表 T5/T7、时间线 T7、WS 1Hz/5Hz+断连横幅 T4、深色中文全局）；§9 部署（T9 Docker/compose + T1 static_dir 容器适配 + T8 deploy.md 全字段表）；瓦片制作（T8 tiles-howto）+ 网格退化（T6，spec §11 风险项的落地）；回放 UI（T5 replaybar + 事件跳转，speed 1/10/60 对应 spec §6）。Plan 3a 携带项四条全落 T1。天空图有数据分支按 epochs.sats_json 的 [{sat,az,el,snr,used}] 约定实现——该字段当前恒空（等真机 stat 流），占位态为主路径。
- **占位符扫描**：无 TBD；T4 的组件 no-op 文件是显式过渡策略（T5-7 替换，T10 整图冒烟兜底），非占位符。
- **类型一致性**：protocol.js 导出名（T3 定义）与 T5/T6/T7 import 一致；store 字段（T4）与各组件读取一致（trails 点 {lat,lon,src,q}、series 五数组、events 含 action）；WsClient 方法名与 replaybar/eventlist 调用一致；MapView/Timelines/Skyplot 构造签名与 app.js 一致；`_FakeApp` 复用自 tests/test_api.py（T10 import 路径 `from tests.test_api import _FakeApp` 要求 tests 有 __init__.py——**没有则 T10 改为把 _FakeApp 复制为本地 fixture**，实现时二选一并在报告说明）。
