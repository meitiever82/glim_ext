# gnss_bringup HMI

`gnss_bringup` 的只读 Web HMI：浏览器实时查看 APP 独立解与 610 融合解、
天空图、诊断事件、曲线与矿区轨迹。代码 fork 自 `rtk-monitor`（基线
8a21851），**去掉了 TCP/CAN 自采集、独立 `rtkrcv` 与本地诊断规则**，只消费
`gnss_bringup` 的 ROS 话题。

设计与任务拆分见：

- `glim_underground/docs/gnss/specs/2026-11-02-gnss-web-hmi.md`
- `glim_underground/docs/gnss/plans/2026-11-02-gnss-web-hmi.md`

## 只读约束（P0）

- 只创建 ROS **订阅者**：无 publisher、无 service/action、不启动/停止任何节点。
- 诊断唯一来源：`/gnss/diagnostics`（`status.name == "gnss_diag"`）。
  本地规则默认关闭；仅当话题缺失且 `diagnosis.local_fallback: true` 时才兜底。
- 默认不落盘：`storage.enabled: false`，事件只在进程内保留最近 5000 条。
- 旧 `rtk-monitor` 采集路径（TCP/CAN/rtkrcv）保留代码但由配置显式关闭，
  HMI 运行环境不需要安装 `python-can`。

## 运行前置

1. ROS 2 Humble 或其他带 `rclpy`、`gnss_msgs`、`diagnostic_msgs` 的 overlay：
   ```bash
   source /opt/ros/humble/setup.bash
   source <driver_ws>/install/setup.bash     # 提供 gnss_msgs
   python3 -c "import rclpy, gnss_msgs"
   ```
2. Python 依赖：`fastapi`、`uvicorn`、`pyyaml`（P0 不需要 `python-can`）。
3. 确认话题存在：
   ```bash
   ros2 topic list | grep -E 'rtk_fix|/stat|diagnostics|rtcm|raw_obs'
   ros2 topic info -v /gnss_cgi610/rtk_fix     # 记录发布端 QoS
   ros2 topic hz /rtkrcv_node/stat
   ```

## 启动

```bash
# 先 source ROS overlay，再“追加”源码路径：PYTHONPATH=src 会覆盖 rclpy 的搜索路径
source /opt/ros/humble/setup.bash
source <你的 overlay>/install/setup.bash
PYTHONPATH=src:$PYTHONPATH python3 -m gnss_hmi.main --config config/gnss_hmi.yaml
# 浏览器打开 http://<车端IP>:8080
```

`ros.use_sim_time: true` + `ros2 bag play --clock` 可做回放核对；
QoS 不匹配时页面链路卡片会提示检查 `ros2 topic info -v`。

## 订阅话题

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/rtkrcv_node/rtk_fix` | `gnss_msgs/RtkFix` | APP 自解（source=app） |
| `/gnss_cgi610/rtk_fix` | `gnss_msgs/RtkFix` | 610 融合（source=fused） |
| `/rtkrcv_node/stat` | `gnss_msgs/RawStream` | `$SAT` 行块 → 天空图 |
| `/gnss/diagnostics` | `DiagnosticArray` | 事件、横幅、status_code |
| `/rtkrcv_node/diagnostics` | `DiagnosticArray` | rtkrcv 健康 |
| `/gnss/rtcm_corrections` | `gnss_msgs/RawStream` | 只统计差分字节速率 |
| `/gnss/raw_obs` | `gnss_msgs/RawStream` | 只统计观测字节速率 |

## 数据映射

- `RtkFix.quality`：0 NONE 灰、1 SINGLE / 2 DGPS 红、3 FLOAT 黄、4 FIXED 绿。
- `sigma_enu` 为 **E/N/U** 顺序，水平 σ = `hypot(E, N)`；前端读 `sigma_h`。
- `ratio == 0`、`gnss_time == 0` 表示源不提供，显示 `—`。
- 接收延迟 = `header.stamp - gnss_time`。

## 测试

```bash
PYTHONPATH=src python3 -m pytest -q tests/test_ros_bridge.py tests/test_config.py
node --test tests_js/*.mjs        # 需要 Node >= 18
```

完整 `pytest` 里仍包含基线遗留的 TCP/CAN 用例，需要 `fastapi`、`python-can`
等旧依赖；P0 验收以 `test_ros_bridge.py` / `test_config.py` 与页面自检为准。

## 与基线的差异

- 包名 `rtk_monitor` → `gnss_hmi`。
- 新增 `src/gnss_hmi/ros_bridge.py`（ROS 只读适配层）。
- `main.py`：按配置跳过 TCP/CAN/rtkrcv/stat 文件路径；新增 1 Hz ROS 状态聚合。
- `web/`：双源徽标、双轨迹、`$SAT` 天空图、链路卡片、延迟/差分速率曲线。
- WS 契约见 `docs/ws-contract.md`。
