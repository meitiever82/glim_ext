# field_replay —— 现场数据整链路回放

把一段实车记录(平台差分 `base.rtcm3`、610 原始输出 `cgi610.dat`、CAN 抓包
`can7.candump.log`)按数据自带时间 1× 实时喂进 GNSS 链路:

```
base.rtcm3 ──TCP:15031──┐
                        ├─ rtcm_bridge ─ /gnss/rtcm_corrections, /gnss/raw_obs ─ rtkrcv_node(rtkrcv, oem4)
cgi610.dat ──TCP:15032──┘                                                          └─ /rtkrcv_node/rtk_fix
can_with_sigma.log ─ canplayer ─ vcan0 ─ gnss_chcnav_can(gnss_cgi610) ─ /gnss_cgi610/rtk_fix
/clock(field_replay 50 Hz)─ gnss_diag_node(use_sim_time)
pos_writer → pos/YYYYMMDD/{can,rtkrcv}.pos;gnss_diag → diag/;record_gnss.sh → bags/
```

## 前置条件

- `vcan0` 已建好且 UP(需要 sudo,维护者操作):
  `sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0`
- `can-utils`(`canplayer`)。
- RTKLIB-EX 2.5.1:`/usr/local/bin/rtkrcv`、`/usr/local/lib/librtklib.so`,源码树
  (默认 `/home/steve/Documents/GitHub/gnss-alg/RTKLIB-2.5.1`,可用 `RTKLIB_SRC` 覆盖)——方案 B
  现场编译 `rnx_nav_to_rtcm.c` 要用它的头文件。
- `glim_ws` 已编译 `gnss_core gnss_bringup`;`driver_ws` 已编译 `gnss_msgs gnss_chcnav`
  (`colcon build --symlink-install --packages-select gnss_msgs gnss_chcnav`)。
- Python 3 + PyYAML(ROS Humble 自带)。

## 用法

```bash
run_field_integration.sh <A|B|dry|dryB> <seg> [duration_s]
```

| 模式 | 差分流 | 星历注入 | 时长 | 运行目录 |
|---|---|---|---|---|
| `A` | `base.rtcm3` 原样 | 无 | 整段 | `<seg>/integration_20260916/run_A` |
| `B` | `base.rtcm3` | `rover.nav` → RTCM3(1019/1020/1042/1045/1046),第一块之前一次、之后每 30 数据秒一次 | 整段 | `run_B` |
| `dry` | 同 A | 无 | 前 `duration_s` 数据秒(默认 90) | `run_dry` |
| `dryB` | 同 B | 同 B | 前 `duration_s` 数据秒(默认 90) | `run_dryB` |

A 的观测流里 610 没有输出 GPS 星历(只有 13 条 Galileo、1 条北斗),rtkrcv 很可能解不出;
B 用当天更早的星历补上,用来区分"链路问题"和"星历缺失"。

脚本做的事:清空运行目录 → 生成补 σ 帧的 CAN 日志副本(和 B 的星历电文)→ 写 `params.yaml`
→ 后台起 `gnss_bringup.launch.py`(`enable_rtkrcv:=true enable_diag:=false enable_cleanup:=false`)、
`gnss_diag_node`(`use_sim_time:=true`)、`gnss_chcnav_can`(节点名 `gnss_cgi610`,`vcan0`,
`timestamp_source:=gps`)、`record_gnss.sh` → 5 s 后前台跑 `field_replay.py` → 依次 SIGINT
录包、诊断、驱动、launch(各等 20 s,超时 SIGKILL 并记录)→ 检查残留进程 → 打印产物清单、
`ros2 bag info` 与各日志尾部。

**开始时刻**:`field_replay.py` 等 `rtcm_bridge` 连上两个端口之后,还要等 rtkrcv 在 rtkrcv_node 的本机端口
(`corr_port`/`obs_port`,默认 15041/15042)上**新建立**一次连接才开始发数据(`--wait-fresh-ports`)。原因:
rtkrcv 的 tcpcli 输入 10 s 没数据就断开、10 s 后才重连(RTKLIB `misc-timeout`/`misc-reconnect` 默认值,
`rtkrcv.conf` 没覆盖),rtkrcv_node 在断开期间收到的字节直接丢弃。不等的话,`rtcm_bridge` 的连接退避
(1+2+4+8 s)正好让回放从第 15 s 开始、落在断开窗口里——2026-09-16 dryB 试跑开头 5 s 数据与第一次星历
注入全部丢失,rtkrcv 到第二次注入(+30 s)才有解;加上等待后第 2 个历元就出解。

排查:`RTKRCV_TRACE_LEVEL=3 run_field_integration.sh dryB <seg> 50` 给 rtkrcv 加 `-t 3`,
trace 写在 `rtkrcv/rtkrcv_*.trace`(level 3 约 1 MB / 50 s)。

隔离:`ROS_DOMAIN_ID=66`、`ROS_LOG_DIR=$RUN/roslog`。**跑回放时不要同时跑 `colcon test`**
(rtkrcv 节点测试用 40–89 的 domain)。

### 产物目录

```
run_<mode>/
  params.yaml             gnss_bringup.yaml 的覆盖:obs_format=oem4、run_dir、pos/diag 根目录、
                          sources=[can, rtkrcv]、startup_grace_s=60
  can_with_sigma.log      CAN 日志副本 + 每周期补 0x326/0x328/0x32B 全零帧(σ 是假的!)
  nav.rtcm3, tools/       (B/dryB)星历电文与编译出的 rnx_nav_to_rtcm
  rtkrcv/                 rtkrcv.conf、rtkrcv_*.stat
  pos/YYYYMMDD/           can.pos、rtkrcv.pos(日期按 gnss_time)
  diag/YYYYMMDD/          events.log、base.pos(按 sim time,即数据日期);diag/base_baseline
  bags/gnss_*/            录包:默认六个话题 + /gnss/diagnostics /rtkrcv_node/diagnostics /clock
  logs/                   run.log(本脚本)、field_replay.log、bringup.log、gnss_diag.log、
                          gnss_chcnav.log、record.log、(B)rnx_nav_to_rtcm_build.log
  roslog/
```

## 为什么要补 σ 帧

这台车的 610 CAN 输出没有 0x326/0x328/0x32B;`gnss_chcnav_can` 要求每个 50 Hz 周期 14 条报文
齐全才发布,缺了就整周期丢弃——**实车上同样一条消息都不会发**。回放用 `can_sigma_patch.py`
在每个周期(从 0x320 开始)最后一条原始帧之后补三条全零帧(时间戳照抄该帧),让驱动的其余链路
能跑通。`can.pos` 的 σ 列因此没有意义。

## 组件

| 文件 | 作用 |
|---|---|
| `field_replay.py` | 编排器:两路 TCP 服务端、canplayer 调度、`/clock`、星历注入、字节核对 |
| `replay_plan.py` | 纯函数:整文件原始字节按帧边界切块、块时刻(帧时间累计最大值)、周期注入、截断 |
| `stream_timing.py` | 纯函数:RTCM3 / NovAtel 分帧与帧时间(Task 2) |
| `candump_log.py` | 纯函数:candump 行解析(Task 2) |
| `can_sigma_patch.py` | 纯函数 + CLI:CAN 日志补 σ 帧 |
| `rnx_nav_to_rtcm.c` | RINEX 星历 → RTCM3 星历电文(运行时编译) |
| `tests/` | `python3 -m unittest discover -s tests`(不用 pytest) |

### field_replay.py 单独使用

```bash
field_replay.py --rtcm <base.rtcm3> --obs <cgi610.dat> [--can-log <log>] [--nav-rtcm <eph.rtcm3>] \
  [--corr-port 15031] [--obs-port 15032] [--can-iface vcan0] [--can-log-iface can7] \
  [--speed 1.0] [--connect-timeout 60] [--reconnect-timeout 30] [--leap 18] \
  [--wait-fresh-ports 15041,15042] [--downstream-timeout 60] [--duration-s N] [--tail-clock-s 10]
```

- 发的是**文件原始字节**,按文件顺序:每块 = 上一帧末尾之后到本帧末尾的全部字节(`cgi610.dat`
  里夹带的 `\n` 与 RTCM3 帧随下一条 NovAtel 帧一起发);块时刻 = 截至本帧的帧时间最大值
  (RANGECMPB 时间最多往回 190 ms)。整段回放结束时打印"已发字节 = 文件大小 + 注入星历字节"。
- `t0` = 三路里最早的数据时刻;两个端口都被 `rtcm_bridge` 连上时记 `W0`;
  `/clock = t0 + (monotonic − W0)·speed`。
- 退出码:0 正常;2 输入/端口错误;3 `rtcm_bridge` 未连接或 `--wait-fresh-ports` 超时;4 断线后未重连;
  5 canplayer 异常;130 信号。
- `--speed` 只影响 TCP 与 `/clock`,canplayer 没有倍速,带 CAN 时应保持 1×。
