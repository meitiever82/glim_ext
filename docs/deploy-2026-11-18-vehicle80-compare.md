# 新旧定位链静态对比测试报告 — nvidia@80（2026-11-18）

## 1. 测试概况

| 项 | 内容 |
|---|---|
| 测试时段 | 2026-11-18 01:30 → 08:37（有效数据 25,632 s ≈ **7.12 h**，≥6h 达标） |
| 地点 | 车辆驻地（44.841382, 89.280398, 627.07 m），露天静止 |
| 对比对象 | **新链** localization@`803-fix`（7dc1663，bynav_adapter + localization_node + metrics）<br>**旧链** localization@`ros2-dev`（78829df，localization_parser） |
| 驱动栈 | finder_ros release a4a98bc1_20261114（按 deploy-sop v1.1 部署，北云 INS） |
| 数据 | 双 bag：`old_20261118_013008`（1.1G）、`new_20261118_013008`（1.2G），30min 分卷 |
| RTK 状态 | INS fixed 96.8% / float 3.2%（42 星，HDOP 0.6） |

## 2. 与标准流程的偏差（均不影响对比有效性）

1. **ENU 原点**：车不在矿区（距矿区原点 ~290 km，超门控 10 km 阈值），两链原点统一改为当前停车位置（44.84138186, 89.28039821, 627.0742）。仅本次无地图静态测试使用；**回矿区部署必须改回现场原点**。
2. **话题重映射**：新链输出改到 `/localization_new/*`（含 fault/heartbeat）。原因：新旧 robominer_msgs 的 `LocalizationData` 定义不同（新版多 `invalid_reason`/`zero_velocity`/`static_std_*` 等字段），同名会 DDS 类型冲突。
3. **相机未参与**：本车相机 OTP/EEPROM 无内参（`gmsl_intrinsics_dump` 如实结果），camchain 用默认 `front-fourcam.yaml` 占位，`--no-camera` 启动；`--no-save` 关驱动栈录像（263GB/h 防撑盘）。
4. **gtsam**：用 `localization/thirdparty/gtsam-4.3a1.zip`（用户提供）源码编译到 `~/loc_test/thirdparty/gtsam-install`。注意车上 `/usr/local/lib` 有一份 20.04 时代的坏 gtsam（依赖 boost 1.71，22.04 上不可加载），新链用 `LD_LIBRARY_PATH` 规避，**建议清理**。
5. **INS 配置补丁**：INS（192.168.7.110，非配置里的 192.168.9.61）保存的配置中 ICOM1（→44445）无数据流，旧链收不到数。在驱动 `receiver_init_commands` 追加 `LOG ICOM1-8 GPGGA/INSPVAXA/CORRIMUDATAA` 后恢复（runtime 生效，INS 重启即回滚）。

## 3. 核心指标（7.12 h 全量）

| 指标 | 旧链（ros2-dev） | 新链（803-fix） | 评价 |
|---|---|---|---|
| 数据完整率 | 99.9995%（1,281,591 帧 @50Hz） | 99.9995%（1,281,478 帧 @50Hz） | 一致 ✓ |
| **is_valid 占比** | **0%（全程无效）** | **96.84%** | ⚠ 见 §4 发现 1 |
| x/y std (m) | 0.0111 / 0.0139 | 0.0111 / 0.0139 | 逐位一致 ✓ |
| 2DRMS (m) | 0.0357 | 0.0357 | 一致 ✓ |
| CEP95 (m) | 0.0566 | 0.0566 | 一致 ✓ |
| 最大水平偏移 (m) | 0.304 | 0.304 | 一致 ✓ |
| z std (m) | 0.0240 | 0.0240 | 一致 ✓ |
| roll/pitch std (°) | 0.0119 / 0.0096 | 0.0119 / 0.0096 | 一致 ✓ |
| yaw std (°) | 0.1220 | 0.1139 | 基本一致 ✓ |
| 速度 std / max (m/s) | 0.0044 / 0.065 | 0.0044 / 0.065 | 一致 ✓ |
| fault 话题消息数 | 180（全部"卫星数不足"） | 82（float 时段正常告警） | ⚠ 见 §4 发现 1 |
| ERROR 日志 | 0 | 0 | ✓ |

## 4. 新旧一致性（时间戳配对，128,149 对）

| 统计量 | 值 |
|---|---|
| Δx / Δy / Δz 均值 | +2.7 µm / +0.4 µm / +1.1 µm |
| Δx / Δy / Δz std | 0.65 / 0.74 / 1.32 mm |
| 水平偏差 p95 / max | **1.1 mm** / 70.8 mm（max 为 float 转换瞬间单点） |
| 3D 偏差 p95 | 2.1 mm |

**结论：两链位置/姿态/速度输出亚毫米级一致，数值 parity 通过。**

## 5. 关键发现

### 发现 1（重要）：旧链全程 is_valid=false —— 数据流缺 GPFPD/HEADINGA（feed 配置问题，非 parser bug）

- 现象：旧链全部 128 万帧 `fault_code=4`（卫星数不足），日志 202 次"定位无效：卫星数不足（rtk=RtkInit(0) svs=0/0）"。
- 根因（已定位到代码）：`beiyun_processor.cpp` 的卫星数硬门控数据源是 **GPFPD 的 nsv1**（`solnsvs`）和 **HEADINGA 的 obs**（`solnsvs2`），**GPGGA 只进 aux 不参与门控**。本次 ICOM 补丁只下发了 GPGGA/INSPVAXA/CORRIMUDATAA，缺 GPFPD/HEADINGA → 两个卫星数源恒 0 → 全程 SatelliteInsufficient。这是 feed 配置不完整，不是 parser 解析错误。
- 生产环境正常的原因：矿区 INS 保存的配置里 44445 推送流包含完整报文集（含 GPFPD/HEADINGA）。
- 已补救：80 车驱动 `bynav_net_tianchinan_nosave.params.yaml`（`config/boonray_tianchinan_80/`）的 `receiver_init_commands` 已补 `LOG ICOM1-8 GPFPD/HEADINGA ONTIME 1`，下次驱动栈启动即生效。
- 新链门控完全正确：`is_valid`=96.84%，与 INS fixed 率 96.84% 精确吻合；float 时段 fault_code=1，告警 82 次，行为符合设计。
- 对本次对比的影响：位置 parity 结论不受影响（两链位置输出均正常）；旧链有效性/告警对比项为"feed 不全，待补流后复测"。

### 发现 2：新链 metrics 工作正常

`LocalizationMetrics`：rtk_fix_ratio=1.0（末段窗口），static_std_horizontal=0.018 m，static_max_deviation=0.287 m，与离线统计口径一致。

### 发现 3（代码问题，建议修）

1. `gnss_bringup_localization.yaml` 的扁平话题键（`localization_topic`/`fault_topic`/`heartbeat_topic`）是**死配置**——`localization_node` 只读 `topics.*` 嵌套键，按 SOP 注释改扁平键不生效（本次踩实）。
2. `gnss_bringup/package.xml` 未声明 `robominer_msgs` 依赖，并行构建会抢跑失败（串行先编 msgs 可绕）。
3. 驱动 release 的 `timesync_setup.sh` 找不到 config 目录（release 布局多一层 `install/`，需软链 `release/share → <pkg>/install/timesync_infra/share`）；`gmsl_intrinsics_dump.py` 未打进 release；脚本缺可执行位（Windows scp 注意）。

## 6. 车辆还原状态（09:00 前完成）

- 双链/录包/驱动栈已停；chrony 已 `enable --now`；robotruck 经 `robotruck_auto_start.sh` 拉起，生产定位 `/localization/localization_data` 恢复 50Hz。
- 生产链当前 `is_valid=false` 属预期（原点=矿区、车在驻地）。
- INS 本次只做了 runtime 配置（未 SAVECONFIG），重启即回生产配置。

## 7. 后续行动项

| # | 事项 | 负责 |
|---|---|---|
| 1 | ~~复现旧链 GPGGA 解析问题~~ 已定位为 feed 缺 GPFPD/HEADINGA；已补入驱动 init 命令，下次起栈后抽 10 分钟确认旧链 is_valid 恢复 | 定位组 |
| 2 | ~~修 gnss_bringup yaml 死键 + package.xml 缺依赖~~ 已修（803-fix 工作区，待提交） | 定位组 |
| 3 | 清理 80 车 `/usr/local/lib` 坏 gtsam | 运维 |
| 4 | 驱动 SOP 修订：timesync 软链、intrinsics 脚本入 release、车队表加 80 | 驱动组 |
| 5 | 条件允许时在矿区原点补一次同项测试（含旧链有效性对比） | 双方 |

数据留存：`nvidia@80:~/loc_test/bags/`（old_/new_20261118_013008，共 2.3G）、`~/loc_test/ana_{old,new}.json`、`ana_pair.json`、日志 `~/loc_test/logs/20261118_012519/`。

## 8. 复测闭环（2026-11-18 中午，nvidia@121）

| 验证项 | 结果 |
|---|---|
| 旧链 is_valid 恢复（GPFPD/HEADINGA 补流后） | ✓ `is_valid=true`、`fault_code=0`、`solnsvs=42/43`、x/y 厘米级，零告警 |
| 新链默认 yaml 死键修复（803-fix 9e339e0） | ✓ 非默认话题名 `/localization/localization_data_test99` 经 yaml 生效；默认名 `/localization/localization_data`+aux+heartbeat+metrics 正常发布 |
| ws_new 串行构建（含新版 msgs overlay） | ✓ 4 包全过（8.5 min） |

121 车已还原（chrony 启用、robotruck 拉起、生产定位恢复）。121 部署复用了 80 的全部经验
（含 ICOM 补丁脚本 `patch_icom_feed.py`、timesync 软链修复），全程约 40 分钟。
