# nvidia@80 静态对比测试计划（新旧定位链）v2

日期：2026-11-17 晚 → 2026-11-18 09:30
目标：803-fix（新，adapter + localization_node）vs ros2-dev（旧，localization_parser）
在 nvidia@80 实车上并行静态测试 ≥ 6 小时，次日 09:30 前输出 MD 对比报告。

## 执行现状（2026-11-18 01:10 更新）

已完成：
- 驱动栈按 finder_ros deploy-sop v1.1 部署到 `~/data/driver_ws`（release a4a98bc1_20261114），
  车队表补 80 行（livox_sn 3JEDM9K001F3341 / ntrip W11_080，实抄自 robotruck）
- B2 sudo 依赖已装（密码由用户提供）；chrony 已停，timesync freerun@eth1 已装
- robotruck 生产栈已停；驱动栈运行中（screen: driver_stack，--no-camera --no-save）
- `/bynav/*` 全话题正常，inspvax 100Hz，RTK **fixed**（pos_type 56 / INS_SOLUTION_GOOD）
- `ws_old`（ros2-dev）构建完成；gtsam **4.3a1**（用 localization/thirdparty 用户提供版）
  编译装到 `~/loc_test/thirdparty/gtsam-install`；`ws_new` 重建中（overlay 新版
  gnss_msgs/robominer_msgs，因车上两处 msg 版本均过旧）
- 启停/巡检脚本：`~/loc_test/{start,stop}_compare_test.sh`、`health_check.sh`

## 与 SOP 的偏差（重要，报告需说明）

1. **车不在矿区**：当前位置 44.84138186, 89.28039821, 627.07（距矿区原点 ~290km，
   超出门控 xyz_threshold=10000m）。**两链 ENU 原点已统一改为当前停车位置**，
   仅用于本次无地图静态对比；回矿区部署必须改回现场原点（铁律2 不变）。
2. **新链输出话题重映射到 `/localization_new/*`**（含 fault/heartbeat）：
   新旧 robominer_msgs 的 LocalizationData 定义不同（新版多 invalid_reason/
   zero_velocity/static_std 等字段），同名话题会造成 DDS 类型冲突，故错开。
3. **相机未参与**：本车 OTP/EEPROM 无内参（gmsl_intrinsics_dump 如实结果），
   camchain.yaml 用默认 front-fourcam.yaml 占位，启动加 `--no-camera`；
   `--no-save` 关闭驱动栈录数据（263GB/h 会撑爆磁盘）。
4. gtsam 等依赖**只用 localization/thirdparty 下用户提供版本**，不自行下载
   （教训：曾误下 Ubuntu pool 版本，已废弃清理）。

## 剩余步骤

1. ws_new 构建完成后冒烟测试（两链各起 5 分钟，验话题/频率/is_valid/fixed 率）
2. 冒烟通过 → 正式起双链 + 录包（30min 分卷，~01:30 起跑 ≥6h 至 ~07:30）
3. 每小时健康检查（cron，进程/bag 增长/磁盘）
4. 08:50 停录汇总，09:30 前出 `docs/deploy-2026-11-18-vehicle80-compare.md`
   指标：数据完整率、RTK 固定率、静态漂移（x/y std、2DRMS、CEP95）、姿态 std、
   速度噪声、is_valid 占比、fault_code 分布、新旧一致性
5. 收尾还原：停双链 → 停驱动栈 → `sudo systemctl enable --now chrony` →
   重新登录桌面拉起 robotruck（09:00 前车要可用）

## 风险

- 本会话需整夜存活（cron 驱动巡检/报告）
- 新旧链消息定义不同，离线对比脚本需按各自版本解析
