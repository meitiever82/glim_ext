# WebSocket 消息契约（`/ws`）

冻结点：本文档随 Plan 3a 最终修复波次一起提交，之后 Plan 3b 前端按此契约开发。
后续如需变更消息形状，须先更新本文档，再改代码与测试
（`tests/test_ws_contract.py` 用 CONTRACT 测试固化了三类消息的 key 集合一致性）。

来源：
- 实时：`src/gnss_hmi/main.py`（`App._diagnosis_tick` 发状态、`App._emit_ros_events`
  发事件），数据来自 `collectors/ros_collector.py` 订阅的 ROS 话题。
- 回放：`src/gnss_hmi/replay.py` 的 `replay_messages()`，从 SQLite
  （`EpochStore`/`EventStore`）重建同形状的消息流。
- 两者的核心不变式：**实时与回放同构**（见 spec §2.1）—— 前端不区分数据来源，
  只按 `type` 分发。回放消息的 key 集合必须与实时消息完全一致；下面列出的是
  *刻意*保留的取值差异，不是 key 差异。

## 1. `status`（1 Hz）

```
{
  "type": "status",
  "t": <float, unix time>,
  "replay": <bool>,
  "stale": <bool, 两路 source 均超过 3 s 无更新>,
  "verdict": {"level": str, "code": str, "message": str},
  "sources": {"app": <source 或 null>, "fused": <source 或 null>},
  "sky": <str, 每颗卫星 [{sat,az,el,snr,used}] 的 JSON 数组>,
  "corr_rate": <rate 或 null>,
  "link": {"corr": <rate 或 null>, "obs": <rate 或 null>,
           "rtkrcv": {"level": str, "message": str} 或 null,
           "topics": {<topic>: <float, 最后消息墙钟时间>}},
  "sol": <sol dict 或 null>,
  "can": <epoch dict 或 null>,
  "gpchc": <epoch dict 或 null>,
  "corr": {"last_t": <float 或 null>, "base_offset_m": <float 或 null>}
}
```

- `sources.app` / `sources.fused`（非 null 时）是 `rtkfix_state` 的输出，key：
  `t, stamp, q, raw_status, lat, lon, alt, sigma_e, sigma_n, sigma_u, age,
  sats, sats_main, sats_aux, heading, heading_valid, ratio, gnss_time, latency,
  present`，再加聚合层写入的 `stale`。`sigma_enu` 按接收机发布的 **E/N/U**
  顺序映射为 `sigma_e/sigma_n/sigma_u`；`ratio<=0`、`gnss_time<=0` 一律为
  null；`heading` 仅在 `heading_valid` 为真时保留。`latency = header.stamp - gnss_time`。
- `sky` 是 `/rtkrcv_node/stat` 的 `$SAT` 流按 `tow` 分历元后的结果；天空图优先
  读它，`sol.sats_json` 仅作旧载荷兼容。
- `rate` 形状为 `{"rate_bps": float|null, "last_t": float|null, "gap_s": float|null,
  "outage": bool}`，由 10 s 滑动窗口计算；`outage` 表示超过 3 s 未收到数据。
- `sol`（非 null 时）固定 12 个 key：
  `t, q, sats, age, ratio, lat, lon, alt, sdn, sde, sdu, sats_json`。
  这是 realtime 与 replay 共享的精确契约。`sol` 优先由 ROS app 源映射
  （`sdn=sigma_n, sde=sigma_e, sdu=sigma_u`），仅当 ROS app 源缺失时才回退到
  本地 solver 的 `.stat` 尾部数据。
- `can`/`gpchc`（非 null 时）是完整的 `Epoch` dataclass（`dataclasses.asdict`）；
  ROS-only 部署下 `can` 回退为 `src="ros_fused"` 的 fused 映射，`speed` 为 null。

### 刻意的取值不对称

- **`verdict`**：回放里恒为 `{"level":"info","code":"replay","message":"回放"}`
  —— 这是一个占位符，表示"这条状态来自回放，不代表当时诊断规则链的真实判定"。
  真实的诊断结论要看同一时间窗内的 `event` 消息（open/close），不要从回放的
  `verdict` 反推诊断状态。
- **`corr`**：回放里恒为 `{"last_t": null, "base_offset_m": null}`。差分链路
  存活时间 / 基站偏移量都是"当下"状态，SQLite 里没有为每一秒存一份历史快照
  （只有 `base_station` 表的离散采样点），回放阶段没有可靠数据源可以重建这两
  个字段，所以统一置空而不是伪造一个值。
- **`sol`/`can`/`gpchc` 的更新粒度**：回放按 `_SNAPSHOT_LOOKBACK_S = 600.0`
  秒向 `t0` 之前回溯做 carry-forward（`replay.py`），实时侧则是
  `sol_stale_s`（配置项，诊断规则链的新鲜度门限，通常几秒级）判定"过期即置
  null"（`main.py::_diagnosis_tick` 的 `sol_stale_s` 检查）。也就是说：同一段
  数据在实时页面上可能因为解算超过新鲜度阈值而显示"无解"，但在回放里因为
  600 秒回溯窗口仍能看到那次快照。这是已知的、可接受的不对称（回放的目标是
  "复现当时写入了什么"，不是"复现当时诊断规则链认为新鲜与否"）。

## 2. `position`（≤5 Hz，仅 `can`/`gpchc` 两路）

```
{
  "type": "position",
  "t": <float>,
  "replay": <bool>,
  "src": "can" | "gpchc",
  "lat": <float>, "lon": <float>,
  "heading": <float 或 null>,
  "q": <int 或 null>,
  "speed": <float 或 null>
}
```

- 实时侧只有 `can`（`_on_can`，≤5Hz 节流发布）会产生 `position`；`gpchc` 路
  当前实时侧不发布位置消息（只落库），但 **回放侧对 `can`/`gpchc` 两路都会
  重建 `position` 消息**（`replay.py` 的 `_SRC_POS = ("can", "gpchc")`）。这是
  刻意的不对称：`gpchc` 在回放里补全轨迹，是因为回放场景（复盘一段行驶）比
  实时监控更需要看到所有可用定位源的完整轨迹；这不代表实时侧的 `gpchc`
  发布行为将来一定不变——如果 Plan 3b 需要实时 `gpchc` 轨迹，需要另起一个
  变更并同步更新本文档与 CONTRACT 测试。
- `rtkrcv` 路不产生 `position` 消息（无论实时还是回放）——rtkrcv 的解算结果
  只出现在 `status.sol` 里。
- ROS-only 部署（P0）不产生 `position`：前端直接按 1 Hz `status.sources.app` /
  `status.sources.fused` 追加双轨，`src` 固定为 `app` / `fused`。

## 3. `event`（诊断事件 open/close）

```
{
  "type": "event",
  "t": <float>,
  "replay": <bool>,
  "action": "open" | "close",
  "event": {"t": <float>, "level": str, "code": str, "message": str}
}
```

- 只有 `etype == "diagnosis"` 的行会产生 `event` 消息。`events` 表还存储
  非诊断行（`etype` 为 `corrections_link`/`raw_obs_link`/
  `gnss_solution_link`/`rtkrcv_sol`/`can_collector`/`web`/... 等，`state` 为
  `connected`/`disconnected`/`crashed`），这些是链路层/进程层的可观测性记录，
  不是面向驾驶员的诊断结论，`replay.py` 与 `report.py` 都显式过滤
  `etype != "diagnosis"` 的行（Plan 3a 最终修复波次 C2）。
- `event.t` 在 `open` 消息里等于事件的开始时间，在 `close` 消息里等于事件的
  结束时间（不是开始时间）——`main.py::_broadcast_event`（实时，来自
  `/gnss/diagnostics` 的 `open_events` 集合差）与 `replay.py` 都遵循这一规则，
  字段集合与语义完全一致。

## 4. `replay_end`（仅回放）

```
{"type": "replay_end", "t": <float, 即回放请求的 t1>}
```

实时流没有对应消息；客户端收到后应视为"这次回放放完了"，UI 层可以据此决定
是否自动切回 live（服务端本身在发完 `replay_end` 后也会自动重新订阅 live，
见 `api.py::run_replay`，客户端可以不用主动发 `{"cmd":"live"}`）。

## 5. `error`（连接层错误，非诊断事件）

```
{"type": "error", "detail": <str>}
```

Plan 3a 最终修复波次新增。两个来源：

1. 客户端发送的 `{"cmd":"replay", ...}` 未通过校验（缺 `t0`/`t1`、非有限数、
   `t1 <= t0`、`speed` 非有限正数）—— `detail: "invalid replay command"`；
   连接保持在当前状态（通常是 live）不受影响。
2. 回放过程中出现未预期异常（例如数据库读取失败）——
   `detail: "replay failed"`；服务端随后自动切回 live，客户端不需要重连。

## 6. `corr` 字段的 null 语义（跨消息统一约定）

`status.corr.last_t` / `status.corr.base_offset_m` 为 `null` 表示"自进程启动
以来从未收到过对应数据"（差分链路从未来过数据 / 从未解析出基站坐标），不是
"当前偏移为 0" 或 "刚刚断开"。Plan 3a 最终修复波次之前，实时侧曾把这两个
字段在未设置时强制置为 `0.0`（I4 修复项），导致前端无法区分"从未收到"与
"偏移恰好为 0"；现在实时与回放两侧都用 `null` 表达"从未收到"，`0.0` 只在真
实收到过一次偏移为 0 的基站坐标时出现。

## 7. `replay` 标记（跨消息统一约定）

`status` / `position` / `event` 三类消息都带一个布尔 `replay` 字段：实时侧恒
为 `false`（`main.py` 三处 `broadcaster.publish`），回放侧恒为 `true`
（`replay.py::replay_messages` 统一包装,`replay_end` 也带）。这不破坏"实时与
回放同构"——键集仍一致,只是取值不同(与 `verdict`/`corr` 同理)。

它解决的问题:服务端回放是**逐条推送**的,客户端点"回到实时"(`{"cmd":"live"}`
+ `store.resumeLive()`)时,已有若干在途回放消息在缓冲/网络里,清空轨迹后才
到达,会在地图上残留几个点、连成一条到实时位置的直线。客户端据此在
`store.applyMessage` 里丢弃"已回到实时后仍到达的回放消息":
`if (m.replay && !s.replaying) return;`。回放模式下 `s.replaying` 为 `true`,
回放消息正常接收;`replay_end` 到达时 `s.replaying` 仍为 `true`(由该消息的
处理器置 `false`),不会被误丢。

## 8. 安全姿态（现状说明，非本次变更范围）

`/ws`、`/api/*`、`/report`、`/tiles/*` 均无鉴权，`web.host` 目前固定绑定
`0.0.0.0`（`main.py::run_forever`）。`POST /api/base_reset` 无需任何凭证即可
把基站坐标重置为最近一次历史值。当前假设是**部署在可信局域网内**（车载
以太网/无外部路由的现场网络），不暴露到公网。若未来需要跑在不可信网络：

- `web.host` 应作为配置项开放（目前 `Config` 里已有 `web.port`，`host` 尚未
  暴露为可配置项，属于已知的后续加固点）。
- `/api/base_reset` 等有副作用的 POST 端点应加最基本的鉴权（如共享密钥/
  局域网内白名单），或至少加操作确认与审计日志。
- `/ws` 目前对连接数/消息速率无限制，局域网单车场景下风险可接受，多租户或
  跑在公网时需要补限流。

这些都不是本波次修复范围（本波次只修正 C1 的回放窗口未校验问题，属于"拒绝
服务"类风险；鉴权缺失是另一类风险，留给部署阶段按实际网络环境决策）。
