# P0 验收记录（只读 Web HMI）

日期：2026-11-03　环境：Linux / ROS 2 Humble / Python 3.10

## 1. 自动化证据（已执行）

| 项目 | 命令 | 结果 |
|---|---|---|
| Python 全量测试 | `PYTHONPATH=src python3 -m pytest -q --ignore=tests/test_docker.py` | **223 passed, 2 skipped** |
| 前端逻辑测试 | `node --test tests_js/*.mjs` | **25 passed** |
| ROS 采集单测 | `tests/test_ros_collector.py`（五档 quality、σ E/N/U、NaN/Inf、断块重组、$SAT 分历元、诊断级别、速率窗口） | 通过 |
| 状态聚合单测 | `tests/test_status_ros.py`（双源/单源缺失/3 s stale/事件去重/$SAT 不进 sol） | 通过 |
| WS 契约 | `tests/test_ws_contract.py`（live 与 replay 顶层键集同构） | 通过 |
| 底图缓存 | `tests/test_tile_manager.py`（瓦片数学、原子切换、保留两版、SHA 校验、断网回退、坏文件回退） | 通过 |

只读静态检查：

```bash
grep -rn "create_publisher\|create_service\|create_client" src/gnss_hmi   # 无匹配
grep -rn "create_subscription" src/gnss_hmi                               # 仅 ros_collector
```

`rtkrcv` 仅在 `rtkrcv.binary` 非空时启动；`config/gnss_hmi.yaml` 中该值为空。

## 2. 只读运行（车端/实验台执行）

```bash
source /opt/ros/humble/setup.bash
source /home/boonraygrn/glim_ws/install/setup.bash
python3 -m gnss_hmi.main config/gnss_hmi.yaml
```

- [ ] `ros2 node info /gnss_hmi`：Publishers 为空，Service Servers 为空，
      Subscribers 含 7 个话题。
- [ ] `ss -tlnp`：无 TCP 客户端连接，无 CAN 线程，无 `rtkrcv` 子进程。
- [ ] `curl -s http://127.0.0.1:8080/api/status`：`sources.app` / `sources.fused`
      至少一路非 null，`verdict` 来自 `/gnss/diagnostics`。
- [ ] 启动日志含 `read-only mode: ros=True tcp_collectors=0 can=0 solver=False storage=False`。

## 3. bag 回放对齐（`use_sim_time` + 临时数据目录）

- [ ] `ros2 bag play <bag> --clock`；HMI 以 `config.replay.example.yaml` 的临时
      `data_root` 启动，确认 `/clock` 驱动页面时间。
- [ ] 同一时间窗内：页面 1 Hz 状态、`.pos`、`events.log` 三条曲线对齐；
      `events.log` 与页面事件列表逐条一致，无 HMI 本地重复事件。
- [ ] 回放含 `$SAT` 时天空图卫星数与 `RtkFix.sats_used` 基本一致；不含时
      显示“等待 $SAT 数据”。

## 4. 真车冒烟

- [ ] 双源在线：APP 自解与 610 融合各自徽标、轨迹、σ 圆正常。
- [ ] 拔掉 610 网线：对应徽标灰化“无数据”，APP 路不受影响；链路卡片 3 s 内
      显示“差分中断”，恢复后自动清除。
- [ ] 拔掉 rtkrcv 上游：`/rtkrcv_node/diagnostics` 事件出现在日志窗。
- [ ] HMI 重启：5 s 内页面恢复；旧 `rtk-monitor` 停止时 8080 无端口冲突。
- [ ] 断网重启：有缓存时底图可用并显示版本；无缓存时显示网格，数据面不阻塞。

## 5. 结论

自动化部分全部通过；第 2–4 节为车端/现场执行项，完成后在本文件勾选并补注
日志与截图路径。
