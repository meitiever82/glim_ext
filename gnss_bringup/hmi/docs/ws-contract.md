# WebSocket 消息契约（`/ws`）

来源：`src/gnss_hmi/main.py`

- ROS 只读模式（P0）：`App._ros_status_loop` 每秒聚合 `RosBridge` 快照，
  经 `Broadcaster` 扇出 `status`；`RosBridge` 的位置回调发 `position`；
  诊断 `open_events` 的差集发 `event`。
- 旧采集模式（仅在 `storage.enabled` 且 TCP/CAN 显式启用时）：沿用
  `_diagnosis_tick` / `_on_can` / `_on_diagnosis_transition` 的旧形状
  （`sol`/`can`/`gpchc`/`corr`），并支持 SQLite 回放。P0 配置不启用。
- 实时与回放的不变式：同一 `type` 的消息 key 集合一致，前端只按 `type` 分发。

## 1. `status`（1 Hz，ROS 只读模式）

```json
{
  "type": "status",
  "t": 1730000000.0,
  "replay": false,
  "sources": {"app": <src>, "fused": <src>},
  "verdict": {
    "level": "ok" | "warning" | "error" | "stale",
    "code": "OK" | "NO_FIX" | "...",
    "message": "中文原文",
    "open_events": ["corr_outage", "..."],
    "divergence_m": 0.42,
    "verdicts": {"corr_outage": "ERROR|差分中断"},
    "startup_grace": false
  },
  "diagnostics": {"gnss": <diag 或 null>, "rtkrcv": <diag 或 null>},
  "corr_rate": {
    "rtcm_bps": 1200.0, "obs_bps": 900.0,
    "rtcm_idle_s": 0.3, "obs_idle_s": 0.3,
    "rtcm_stale": false, "obs_stale": false
  },
  "satellites": [{"sat": "G01", "az": 45.0, "el": 30.0, "snr": 45.0, "used": true}],
  "sat_count": 24,
  "links": [{"topic": "/gnss/diagnostics", "kind": "diagnostic", "qos": "auto",
             "source": "", "age_s": 0.4, "state": "ok"}],
  "stale": false,
  "startup": false,
  "ros_error": null
}
```

`<src>`（`sources.app` / `sources.fused`）固定字段：

```json
{
  "available": true,
  "t_recv": 1730000000.0,
  "stamp": 1730000000.05,
  "gnss_time": 1730000000.0,
  "latency_s": 0.05,
  "quality": 4,
  "raw_status": 0,
  "lat": 44.5, "lon": 90.25, "alt": 1000.0,
  "sigma_e": 0.010, "sigma_n": 0.012, "sigma_u": 0.030, "sigma_h": 0.0156,
  "age_s": 1.2, "sats": 24, "sats_main": 20, "sats_aux": 4,
  "ratio": 9.5,
  "heading": 123.4, "heading_valid": true,
  "speed": null,
  "stale": false,
  "reason": "无数据"
}
```

约定：

- `quality` 直接使用 `RtkFix.quality` 0–4（0 NONE、1 SINGLE、2 DGPS、
  3 FLOAT、4 FIXED），前端不再做 old rtkrcv/610 档位换算。
- `sigma_*` 来自 `sigma_enu`（**E/N/U 顺序**）；`sigma_h = hypot(E, N)`。
  不使用 rtn-monitor 的 `sdn/sde` 命名。
- `ratio == 0`、`gnss_time == 0` 表示源不提供 → `null`（页面显示 `—`）。
- `available` 由 `stale_s`（默认 3 s）判定；无数据时为 `false` 并带
  `reason`。**保留最后位置字段**供地图显示，但不追加新轨迹点。
- `link.state`：`ok` / `stale`（超 `stale_s`）/ `waiting`（启动宽限内）/
  `no_data`（宽限后从未收到；提示检查 `ros2 topic info -v` 的 QoS）。
- `verdict` 只来自 `/gnss/diagnostics`（`status.name == "gnss_diag"`）与
  `/rtkrcv_node/diagnostics`，取最严重级别；HMI 不重跑规则。
- 宽限期内不把“未收到”渲染成故障；`startup_grace` 原样透传。

旧采集模式仍会给出 `sol` / `can` / `gpchc` / `corr` 字段（见基线契约），
P0 部署不启用，前端也不读取它们。

## 2. `position`（≤5 Hz，`app` / `fused` 两路）

```json
{
  "type": "position",
  "t": 1730000000.0,
  "replay": false,
  "src": "app" | "fused",
  "lat": 44.5,
  "lon": 90.25,
  "heading": 123.4,
  "q": 4,
  "speed": null
}
```

- `heading` 无效（`heading_valid == false`）时为 `null`，前端不画箭头。
- 两路轨迹独立追加；`store.trails.app` / `store.trails.fused` 互不覆盖。

## 3. `event`（诊断事件 open/close）

```json
{
  "type": "event",
  "t": 1730000000.0,
  "replay": false,
  "action": "open" | "close",
  "event": {"t": 1730000000.0, "level": "critical", "code": "corr_outage",
            "message": "差分中断"}
}
```

- `open_events` 中新出现的规则码 → `open`；消失的 → `close`
  （`message: "已恢复"`）。同一规则码不会重复 open。
- `level` 由 `verdict.<code>` 前的 `ERROR/WARN/STALE/OK` 映射为
  `critical/warning/warning/info`。
- 差分/观测链路断开不另造事件，由 `corr_rate.*_stale` 与链路卡片展示。

## 4. `replay_end` / `replay` 命令（仅 `storage.enabled`）

```json
{"type": "replay_end", "t": 1730003600.0}
```

- `storage.enabled: false` 时服务端对 `{"cmd":"replay",...}` 返回
  `{"type":"error","detail":"replay unavailable (storage disabled)"}`，
  连接保持 live。
- 回放消息统一带 `"replay": true`；客户端在 `store.applyMessage` 中丢弃
  “已回到实时后仍到达的回放消息”。

## 5. `error`

```json
{"type": "error", "detail": "invalid replay command"}
```

来源：回放命令校验失败、回放异常、或存储关闭时请求回放。

## 6. null 语义

`null` 表示“源不提供/从未收到”，不是 0：`ratio`、`gnss_time`、
`latency_s`、`sigma_*`、`age_s`、`speed`、`heading` 均遵循此约定。
