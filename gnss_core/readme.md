# gnss_core

无 ROS、无 GLIM 依赖的 C++17 GNSS 算法库。

它**不做** RTK 解算（那是 RTKLIB/rtkrcv 的事），**不做**通信与界面；它做两件事：

1. 把 GNSS 定位解变成**能进因子图的约束**（质量门控、坐标转换、杆臂、噪声模型）；
2. 回答**这条解能不能信、出了什么问题**（在线诊断、事件记录、离线报告）。

同一个库被三类消费者共用：`gnss_bringup` 的 ROS 节点、`glim_ext` 的融合模块
（`rtk_odometry` / `rtk_global`，以 `.so` 挂进 GLIM）、以及一组离线工具。

---

## 1. 模块总览

### 1.1 基础数据与坐标

| 头文件 | 做什么 |
|---|---|
| `types.hpp` | 源无关的 `RtkFixSample`、`Quality{NONE,SINGLE,DGPS,FLOAT,FIXED}`、`effective_stamp()`（在 header.stamp 与板卡时间之间选有效时间） |
| `geodetic.hpp` | WGS-84 经纬高 ↔ 局部 ENU（GeographicLib `LocalCartesian`）、两点测地线距离 |
| `pos_io.hpp` | RTKLIB `.pos` 读写：统一换算成 UTC unix 秒（GPST 减闰秒）、`PosWriter` 追加写 + 毫秒键去重 + 开机截断断电留下的半行 |
| `rtkstat.hpp` | `parse_llh_solution()`（rtkrcv 解流行 → `PosRecord`）、`$SAT` 行解析、按历元聚合、30 s 周跳窗口 |

时间约定贯穿全库：**一律是"数据时间"（UTC unix 秒），不是壁钟**。插值、剪枝、去重都按它算。

### 1.2 融合约束（给 GTSAM）

| 头文件 | 做什么 |
|---|---|
| `rtk_fix_buffer.hpp` | 按 stamp 递增的样本环形缓冲；线性插值位置/σ，航向按最短弧插值，质量取两端较差者，跨大缺口拒绝插值 |
| `rtk_noise_policy.hpp` | 质量门控：质量档、差分龄期、卫星数、σ 有限性；按质量分档缩放 σ + 下界 + 垂直放大 + huber/cauchy 鲁棒核，输出 `gtsam::SharedNoiseModel` 或带原因的拒绝 |
| `frame_aligner.hpp` | 用成对 (submap 位置, ENU) 做 2D Umeyama，bootstrap `T_world_enu`，**之后冻结** |
| `gauge_registry.hpp` | 进程级共享 gauge：先 bootstrap 者发布、后来者采用（`shared_gauge` 实验开关） |
| `anchor_pipeline.hpp` | 一条完整链路：样本缓冲 → 按 anchor 时刻插值 → 噪声门控 → ENU → 求解 `T_world_enu` → 输出 `Constraint`；含"gauge 未定先暂存、bootstrap 后补发"、按原因分类的统计、可选绝对航向 |
| `relative_pipeline.hpp` | 相邻历元的水平位移约束，独立（更短基线）yaw bootstrap，在绝对锚点可用前先给 smoother 形状约束 |
| `antenna_prior_factor.hpp` | 带杆臂的位置先验因子：`h(X) = X * body_point`，残差对 world 位置 |
| `heading_prior_factor.hpp` | 双天线绝对航向一元因子（yaw 残差 + 回绕，有限差分 Jacobian） |
| `relative_translation_factor.hpp` | 两帧之间的二元水平位移因子（不约束旋转、不看高程） |
| `factor_compat.hpp` | GTSAM 4.2 / 4.3 的 `OptionalMatrixArg` 等兼容层 |

主线：

```
RtkFixSample ─► RtkFixBuffer ─► 插值 ─► RtkNoisePolicy ─► LlaToEnu
                                               │
                                               ▼
                                    FrameAligner (T_world_enu, 一次冻结)
                                               │
                                               ▼
                                        Constraint ─► AntennaPriorFactor ─► 图
```

质量门控是**硬门限**：低于 `min_quality`（默认 FLOAT）、差分龄期超限、卫星数不足的样本直接拒绝，
不"降权进图"。通过门限的样本按 `quality_sigma_scale` 缩放板卡 σ，并施加 σ 下界
（杆臂未标定期间默认 1 m，标定后可降到 cm 级）。

### 1.3 在线诊断

| 头文件 | 做什么 |
|---|---|
| `rtcm.hpp` | CRC-24Q、增量 `RtcmFramer`（任意切分字节流 → 完整帧，CRC 失败只丢 1 字节重新同步）、RTCM 1005/1006 基站坐标解析 |
| `diagnosis.hpp` | 纯规则链 `evaluate_rules()`：`no_data`、`corr_outage`、`base_shift`、`abs_ref_shift`、`low_sats`、`multipath`、`ambiguity`、`cycle_slip`、`device_divergence`，最后追加状态（`rtk_fixed` / `not_fixed` / `no_solution`） |
| `divergence_monitor.hpp` | 610 融合解与 rtkrcv 独立解的水平偏差阈值：`divergence_sigma × max(5 cm 下限, 经验基线, 独立解本拍 σ)`，带预热、held 基线、按历元/到达时刻配对 |
| `event_book.hpp` | 每个规则码一个带关闭迟滞的事件跟踪器，输出开/关变更与峰值指标 |
| `base_station_monitor.hpp` | 首次运行取 1005/1006 各轴中位数作基线，之后报告 ECEF 位移；坐标变化时标记需要写 `base.pos`；支持运维确认搬迁后 `reset()` |
| `diagnosis_engine.hpp` | 把上面几件串起来：喂差分字节/两路解/`$SAT` 行，每秒 `tick()` 出结论、事件与偏差状态 |
| `diag_io.hpp` | `events.log` / `base.pos` 的写出行格式与读回解析（含半行、粘连行的容错） |
| `retention.hpp` | 按天 + 磁盘水位清理：从旧到新删，今天与最新一项永不删；查不到磁盘占用时能与"用了 0%"区分 |

诊断链路：

```
RTCM 字节 ─► RtcmFramer ─► BaseStationMonitor (1005/1006)
rtkrcv llh ─► SolutionSample ─┐
610 RtkFix ─► SolutionSample ─┼─► DiagnosisEngine ─► verdict / EventTransition / DivergenceState
$SAT 行 ────► SatStat + SlipWindow ─┘
```

### 1.4 离线报告与验证

| 文件 | 做什么 |
|---|---|
| `report_inputs.hpp/.cpp` | 从 `<root>/YYYYMMDD/` 装载时间窗内的 `.pos`、`events.log`、`base.pos`，配对开/关事件，多读前一天以找回跨窗事件 |
| `report_stats.hpp/.cpp` | 固定解占比、分小时固定率、按质量分档统计、610 vs rtkrcv 偏差、绝对基准校验、基站稳定性、事件汇总、轨迹分段 |
| `report_svg.hpp/.cpp` | 纯字符串生成内联 SVG（折线图、柱状图、轨迹、事件标记） |
| `report_html.hpp/.cpp` | 单个自包含 HTML（内联 CSS/SVG，无脚本无外部资源），浏览器打印为 PDF |
| `report_cli.hpp/.cpp` | `gnss_report` 的参数解析与执行（库内实现，`tools/gnss_report.cpp` 只是 main） |
| `synth.hpp/.cpp` | 轨迹（TUM）→ 合成 RTK 观测，支持错误固定、差分龄期停滞、降级浮点三类注入 |
| `lever_arm_estimator.hpp/.cpp` | 联合估计 `lever_imu`、`T_world_enu`、时间偏移：对 Δt 网格搜索，内层交替最小二乘，输出可观性判断 |
| `trajectory_compare.hpp/.cpp` | 两条 `.pos` 按时间配对，按质量档给 RMSE 与三种 "实际误差 / 板卡 σ" 口径 |
| `judge.hpp/.cpp` | 定位门控判决纯函数（从 `localization_parser` 抽出，golden parity 对齐）；**当前未接入任何运行路径** |

`tools/` 下的可执行与脚本（用法与实测数字见 [tools/README.md](tools/README.md)）：

- `calibrate_sigma_scale` —— 实测标定 `quality_sigma_scale`
- `estimate_lever_arm` —— 杆臂/时间偏移
- `synth_rtk_fix` —— 合成观测与故障注入
- `gnss_report` —— 生成 HTML 定位报告
- `export_bag_to_pos.py`、`pos_to_rtkfix_bag.py`、`traj_rmse.py`、`make_synthetic_pos.py`、`run_injection_suite.sh`

---

## 2. 谁在用它

| 消费者 | 用到的部分 |
|---|---|
| `gnss_bringup/rtkrcv_node` | `rtkstat::parse_llh_solution` |
| `gnss_bringup/pos_writer` | `pos_io::PosWriter`、`PosRecord` |
| `gnss_bringup/gnss_diag` | `diagnosis_engine`（内部含 `diagnosis` / `rtcm` / `divergence_monitor` / `event_book`）、`diag_io`、`base_station_monitor`、`retention` |
| `gnss_bringup/gnss_cleanup` | `retention` |
| `glim_ext` rtk_odometry / rtk_global | `AnchorPipeline`、`RelativePipeline`、三个因子、`GaugeRegistry`、`rtk_noise_policy` |
| 离线 | `tools/` 全部 |

---

## 3. 构建与测试

依赖：C++17、Eigen3、GTSAM（4.2 或 4.3，不锁版本）、GeographicLib。

同一份源码支持两条构建路径：

```bash
# colcon（Orin / ROS 环境，有 ament）
colcon build --packages-select gnss_core

# 纯 CMake（云端 / CI，不装 ROS）
cmake -S gnss_core -B build && cmake --build build -j
ctest --test-dir build --output-on-failure
```

测试：30 个 GTest 二进制 + 1 个 Python unittest（`tools/export_bag_to_pos.py`）。
`BUILD_TESTING=ON` 时才会编译并安装 `pos_io_test_hooks.hpp` 与对应注入点，
正式发布的 `libgnss_core.so` 导出符号里不含测试钩子。

---

## 4. 设计约定

- **无 ROS / 无 GLIM / 无线程**：库只做纯计算。线程、队列、时间源、gtsam key 映射、
  日志都由壳负责。`Push*` 进来的数据假定调用方已做 finite 与时间戳合理性过滤。
- **时间用数据时间**：`prune` / 插值 / 去重的 `now` 是样本时间，不是 `rclcpp::Clock::now()`。
- **gauge 一次冻结**：`T_world_enu` bootstrap 后不随优化推进重估——否则新因子按新基准、
  旧因子按旧基准，同一张图里会有两套坐标基准。
- **失败要响**：配置非法抛 `std::invalid_argument` 并以字段名开头；文件读错误抛
  `std::runtime_error`，绝不把"读到一半"当成读完；写不进去返回 false 而不是静默丢数据。
- **σ 顺序是历史坑**：`PosRecord.sdne` 是 RTKLIB 的 `(sdn, sde, sdu)` = N/E/U，
  而 `RtkFix.sigma_enu` 是 E/N/U，跨边界必须交换前两项。
- **不在本库做的事**：RTK 解算、NTRIP/TCP 传输、ROS 话题、Web HMI、`.pos` 文件轮转目录策略
  （目录与保留策略在 `gnss_bringup`，本库只提供 `PosWriter` 与 `retention` 的纯逻辑）。

---

## 5. 相关文档

- [tools/README.md](tools/README.md) —— 标定、合成注入、杆臂估计、报告工具的完整用法
- `glim_underground/docs/gnss/specs/` —— 各轮设计规格（诊断规则口径、shared-gauge、阶段二相对约束）
- `glim_underground/docs/gnss/plans/` —— 各轮实现计划与设计决定
