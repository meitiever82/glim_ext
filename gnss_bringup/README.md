# gnss_bringup

三个 ROS2 节点,构成 GNSS 数据面的第一段:

- **`rtcm_bridge`**——把裸 TCP 字节流(平台差分、板卡原始观测)桥接到 ROS 话题
  (`gnss_msgs/RawStream`)。不做任何解析,支持连接(`listen=false`)或监听
  (`listen=true`)两种方向,断线自动重连/继续等待。
- **`rtkrcv_node`**——生成 `rtkrcv.conf`、监管 RTKLIB `rtkrcv` 子进程、把它的
  llh 解算流转成 `gnss_msgs/RtkFix` 发布、把 `$SAT` 状态行原样转发到 `~/stat`
  供后续诊断消费。
- **`pos_writer`**——订阅 N 路 `gnss_msgs/RtkFix`,每路各自按 1 Hz 抽稀、按
  UTC 自然日轮转,写成 RTKLIB 兼容的 `.pos` 文本摘要(见下面「`.pos` 输出」
  一节)。

全量原始流(两路裸流 + 各路 `RtkFix` + `~/stat`)的记录另见「rosbag2 录制」
一节的 `scripts/record_gnss.sh`。

## 构建(2026-09-12 更新:不再需要 source driver_ws)

**新机器 / 新克隆,第一次 `colcon build` 之前必须先跑一次**:

```bash
bash src/glim_ext/setup_workspace.sh
```

这个脚本在 `glim_ws/src/` 下建三个符号链接:`gnss_core`、`gnss_bringup`
(两者源码正本都在 `src/glim_ext/` 下,colcon 发现了 `glim_ext` 这个包之后不会
再往它里面递归,所以这两个子包不建链接就永远不会被 colcon 看到),以及
**`gnss_msgs`**(源码正本在 `driver_ws` 那边的 `finder_ros/drivers/gnss_msgs`,
默认路径可用 `GNSS_MSGS_SRC` 环境变量覆盖)。三个链接建好之后,**构建
`glim_ws` 不再需要先 `source driver_ws` 的 overlay**——`gnss_msgs` 的源码已经
直接链接进本工作区,`colcon build` 会自己编出一份 `install/gnss_msgs`。

> **这个选择的代价,必须显眼地写在这里**:`gnss_msgs` 因此会在两个独立的
> workspace 里各编一份——`driver_ws` 那边为了 `gnss_chcnav`(即下文
> `/gnss_cgi610/rtk_fix` 的发布者)也在编它自己的 `gnss_msgs`。这意味着
> **改过任何 `.msg` 文件之后,`glim_ws` 和 `driver_ws` 两个 workspace 都必须
> 重新构建**,不能只改一边——否则一边用新的消息结构发布、另一边还在用旧的
> 消息结构订阅,而 **ROS2 Humble 在这种情况下的表现是 DDS 静默不匹配**:
> 不报错、不警告、rqt/`ros2 topic info` 看起来一切正常,话题就是收不到任何
> 数据,现场极难排查。轮 1 给 `RtkFix` 加 `gnss_time` 字段时就正好踩中过这个
> 场景。**检查清单**:改了 `gnss_msgs/msg/*.msg` 之后,`glim_ws` 和
> `driver_ws` 都要 `colcon build --packages-select gnss_msgs`(及其下游包)
> 再重启相关节点,两边缺一不可。

## 安装 RTKLIB-EX 2.5.1(`rtkrcv`)

`rtkrcv_node` 需要 **RTKLIB-EX 2.5.1**(rtklibexplorer 维护,原 demo5)。
**不要用 `apt install rtklib`**:Ubuntu 22.04 源里是 Takasu 原版 2.4.3 b34,不认 `-nc`,
遇到就打印用法并以 0 退出,节点会陷入崩溃循环。

```bash
# 源码:https://github.com/rtklibexplorer/RTKLIB/releases/tag/v2.5.1
cd RTKLIB-2.5.1
# 只要命令行工具:关掉 Qt(系统 Qt6 缺 SerialPort 模块会让配置失败)。
# 需要 GUI 时去掉最后一个 -D,改传 -DCMAKE_PREFIX_PATH=<带 SerialPort 的 Qt6 目录>。
cmake --fresh -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_QT=TRUE
cmake --build build -j"$(nproc)"
sudo cmake --install build
sudo ldconfig        # 必须:librtklib.so 装在 /usr/local/lib,不刷新缓存 rtkrcv 起不来
rtkrcv --version     # 应输出:rtkrcv RTKLIB EX 2.5.1
```

`binary` 参数默认 `rtkrcv`,节点按 `PATH` 查找;找不到时节点启动即失败,日志里带着当时的 `PATH`。

## 快速开始

```bash
source install/setup.bash
ros2 launch gnss_bringup gnss_bringup.launch.py
```

参数集中在 `config/gnss_bringup.yaml`,launch 文件默认加载它;可用
`params_file:=<path>` 覆盖整份文件,或 `-p <name>:=<value>` 覆盖单个参数。

`enable_rtkrcv:=false` 只起 `rtcm_bridge`(RTKLIB 未安装、或只想验证桥接这一段时用,
见下面"不装 RTKLIB 也能跑起来"一节)。`enable_pos_writer:=false` 关掉 `.pos`
落盘(默认 `true`,与 `enable_rtkrcv` 相反——`pos_writer` 只依赖
`gnss_msgs/RtkFix`,不依赖 RTKLIB 本身,默认随桥一起起来)。

## 部署要求(必读):`rtkrcv_node` 必须跑在 systemd + cgroup `KillMode` 下

`ProcessSupervisor` 用 `setsid()` 把 `rtkrcv` 放进独立进程组,这是为了让
`stop()` 能对整组发信号收尾派生出来的孙进程——这个决定本身是对的。但它的
代价是:`rtkrcv` 子进程只在父进程走**正常的 `stop()`/析构路径**时才会被
一起收尾;父进程如果是被 `kill -9`(或者任何不给它机会跑完析构函数的方式,
比如 OOM killer)杀掉的,`rtkrcv` 子进程会被 init 收养、继续独立运行,
变成一个孤儿进程。

已实测复现的后果——而且是那种**看起来一切正常、实则数据完全错误**的最坏
情况(下面两条代码层面的修复落地之前的行为):

1. `kill -9 <rtkrcv_node 的 pid>`。
2. 孤儿 `rtkrcv` 继续拿着 `sol_port`(以及 `corr_port`/`obs_port`)当
   TCP 服务端挂着。
3. 重新拉起 `rtkrcv_node`:它自己的 `ProcessSupervisor` 会尝试再起一个
   **新的** `rtkrcv`,而这个新进程会因为端口被孤儿占着而绑定失败、不断
   崩溃重启——但这不是唯一的坏结果。
4. 更隐蔽的是:`rtkrcv_node` 里负责去**连**解算输出端口(`sol_port`)的
   `TcpStream` 是客户端角色,它会连上"当前监听在这个端口上的任何东西"——
   也就是那个孤儿 `rtkrcv`,而不是新起的那个。孤儿继续吐着它自己那份
   (可能早已过时、来自另一次运行的)解算结果,`rtkrcv_node` 原样转发,
   只是套上一个全新的 `header.stamp`——下游拿到的是一条**时间戳新鲜、
   内容却是旧的/错的定位解**,没有任何报错或状态异常可以据此发现问题。

**现在的代码层面已经落地两道防线**(final-fix-wave 之前只有下面的
systemd/`pgrep` 规避,现在两者都在):

1. **`prctl(PR_SET_PDEATHSIG)`**(`process_supervisor.cpp`):`rtkrcv` 的
   fork() 发生之后,子进程立刻给自己设置"父进程死亡时收到 SIGTERM"——
   这样父进程被 `kill -9` 杀死时,内核会顺带把 `rtkrcv` 也 SIGTERM 掉,
   不再变成孤儿。注意 `PR_SET_PDEATHSIG` 是**线程级**语义("父进程"实际
   指"创建这个子进程的那个线程"),`ProcessSupervisor` 的 fork() 因此固定
   跑在一个专用线程上,细节见该文件里的注释。
2. **启动时拒绝**(`rtkrcv_node.cpp` 的 `check_sol_port_free()`,排在
   `read_params()` 之后、`write_conf()`/起本地监听之前执行):起 `rtkrcv`
   之前先探测 `sol_port` 是不是已经有人在监听,是的话直接拒绝这一次启动,
   打印:

   > `sol_port=<port> 已经有人在监听——很可能是上一次 rtkrcv_node 异常退出`
   > `(kill -9/OOM)遗留的孤儿 rtkrcv 进程,也可能是误开的第二个实例。`
   > `继续启动会导致新起的 rtkrcv 绑不上这个端口,而本节点的 TcpStream`
   > `转而连上那个孤儿,把它的陈旧解当新鲜数据发布出去。请先用`
   > `` `pgrep -a rtkrcv` `` `排查并手动杀掉残留进程,确认端口空闲后再重启本节点。`

   这条路径不会被记成"配置有误"(main() 里单独识别成
   `OrphanPortError`,措辞是"sol_port 疑似被残留的孤儿 rtkrcv 占用"),
   退出码和"打一行 RCLCPP_ERROR + 非零退出"这个行为不变,只是不会误导
   操作人员去检查参数。探测本身失败(比如 fd 耗尽)同样拒绝启动,不会
   把"探测不出来"悄悄当成"端口空闲"。

**这两道防线仍然覆盖不到的残留情况**,systemd + cgroup `KillMode` 这条
规避依然需要保留作为兜底:

- **旧版本编译出来的、还没打 `PR_SET_PDEATHSIG` 补丁的 `rtkrcv_node`**
  留下的孤儿——`prctl` 是运行时行为,取决于当时跑的是哪个版本的二进制,
  不能靠现在的代码追溯性地保护过去启动的进程。
- **一个继承了 `sol_port` 监听 socket 的、`rtkrcv` 自己的子孙进程**——
  `PR_SET_PDEATHSIG` 只保证 `rtkrcv` 本身收到信号,如果它自己派生出的
  某个子进程意外继承了监听 fd 并且在 `rtkrcv` 退出后继续存活,启动时的
  端口探测依然会命中、依然会正确拒绝启动,但这种情况下"先 `pgrep -a
  rtkrcv` 确认没有残留"的人工检查依然是唯一能分辨"这是不是同一类问题"
  的手段。

因此仍然建议:

- `rtkrcv_node` 由 systemd 管理,并且 unit 文件里配置基于 cgroup 的
  `KillMode=control-group`(而不是默认的 `KillMode=process`)——这样
  systemd 停止/重启这个服务时,会对整个 cgroup(包含任何残留进程)发
  信号,而不只是对 `rtkrcv_node` 自己的 pid,是比 `PR_SET_PDEATHSIG`
  更彻底的兜底。
- 启动前用 `pgrep -a rtkrcv` 确认没有残留进程仍然是好习惯——现在即使
  忘了做这一步,`check_sol_port_free()` 也会在端口冲突时直接拒绝启动
  并打印上面那条消息,不会再静默造成"看起来正常运行,却在发布过时/
  错误的解"这种最难排查的故障。

## 现场待确认(spec §13 P0)

以下四项在现场设备到位前只能先用占位值。协议本身已经确认——`rtcm_bridge` 是裸
TCP、不含 NTRIP(参考实现 rtk-monitor 全仓无 NTRIP);待确认的只是端点、方向、
和两个 `inpstr*-format` 的取值。**这四项都只需要改 `config/gnss_bringup.yaml`
里的值,不需要改任何代码。**

| # | 待确认                               | 影响                             | 现在的取值                            | 确认后怎么改                                                                                         |
| - | ------------------------------------ | -------------------------------- | ------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| 1 | 平台差分的 IP:Port,以及连接方向      | `rtcm_bridge` 连不上就没有差分 | `127.0.0.1:15031`, `listen=false` | 只改`config/gnss_bringup.yaml` 里 `rtcm_bridge.rtcm_corrections` 的 `host`/`port`/`listen` |
| 2 | 板卡原始观测的端点                   | `rtkrcv` 无观测无法解算        | `127.0.0.1:15032`, `listen=false` | 只改`config/gnss_bringup.yaml` 里 `rtcm_bridge.raw_obs` 的 `host`/`port`/`listen`          |
| 3 | 板卡原始观测格式(`inpstr1-format`) | 格式错`rtkrcv` 解不出          | `rtcm3`(沿用 rtk-monitor 假定)      | 只改`config/gnss_bringup.yaml` 里 `rtkrcv_node.obs_format`                                       |
| 4 | 平台差分格式(`inpstr2-format`)     | 同上                             | `rtcm3`                             | 只改`config/gnss_bringup.yaml` 里 `rtkrcv_node.corr_format`                                      |

### 两路裸流:同一个消息类型,靠话题名区分,不是靠类型区分

`rtcm_corrections`(平台差分)与 `raw_obs`(板卡原始观测)是两路**完全独立**的
数据,但在 ROS 层面用的是**同一个消息类型** `gnss_msgs/RawStream`——`rtcm_bridge`
本身不解析任何字节,只是把 TCP 字节流原样搬进 ROS 话题,所以两路消息在类型层面
看起来一模一样。区分它们的**只有话题名**:

- `/gnss/rtcm_corrections`——平台差分修正数据。
- `/gnss/raw_obs`——板卡自己的原始观测数据。

两路的实际编码格式(是不是 RTCM3、还是别的格式)不体现在 `RawStream` 类型里,
而是由下游 `rtkrcv_node` 的两个参数分别声明:`corr_format` 对应
`rtcm_corrections`,`obs_format` 对应 `raw_obs`(见上面现场待确认表的第 3、4
项)。订阅这两个话题时,**不要指望消息内容本身能告诉你这是哪一路**——永远按
话题名区分,`rtcm_bridge` 也永远不会把两路弄混(每一路在配置里各自绑定固定的
`topic`)。

### 只有原始观测、没有差分时怎么配

如果现场只接上了板卡原始观测、平台差分还没到位,可以只起 `raw_obs` 这一路
(已实测):

```yaml
rtcm_bridge:
  ros__parameters:
    streams: ["raw_obs"]
    # 只保留 raw_obs 这一段配置,删掉 rtcm_corrections 那一段
```

这条链路是**通的**:`rtkrcv` 在没有差分修正的情况下依然会解算并输出结果,但
**只能是单点解**(RTKLIB `Q=5` → `gnss_msgs/RtkFix` 里归一化成
`QUALITY_SINGLE`)。约束模块(定位/建图侧消费 `RtkFix` 的模块,例如
`config_rtk_odometry.json`/`config_rtk_global.json` 里的 GNSS 约束)默认的
`min_quality` 会把 `QUALITY_SINGLE` 整体拒掉——**这是设计意图,不是故障**:
链路能跑通、`RtkFix` 也在正常发布,只是不会产生任何 GNSS 约束。如果现场确实
只有单点解能用、又想让约束生效,需要现场评估精度后主动调低对应约束模块的
`min_quality`,而不是当成这里的 bug 来排查。

## 参数说明

`config/gnss_bringup.yaml` 里的大部分参数名直接对应 RTKLIB 的 conf 键
(`pos_mode`、`navsys`、`elmask`、`ar_mode` 等),或是显而易见的端点配置
(`host`/`port`/`listen`/`topic`)。ROS 对未声明的 YAML key 是**静默忽略**的——
写错一个参数名不会报错,只是那个设置根本没生效——所以下面两组容易被漏掉的参数
专门说明一下:

- **`base_pos_type`(默认 `rtcm`)**——基准站坐标来源,对应 conf 的 `ant2-postype`。
  `rtcm` 要求平台差分流带 RTCM 1005/1006;没有时改成 `single`(基准站单点解,精度差)。
  **这一项不写时 rtkrcv 默认坐标 0,0,0,RTK 一条解都不输出**。`bds_ar_mode`/`glo_ar_mode`
  是北斗/GLONASS 模糊度固定开关,默认值与 RTKLIB-EX 2.5.1 一致,现场按固定率调整。
  所有枚举参数在节点启动时按 RTKLIB-EX 2.5.1 的取值表校验,写错直接拒绝启动——
  rtkrcv 自己遇到非法取值只会悄悄回落到默认值继续跑。
- **`leap_seconds`(默认 18)**——GPST 与 UTC 之间的闰秒偏移量。这个值不是常量,
  IERS 每次宣布插入新闰秒后都需要手动更新;`rtkrcv_node` 用它把解算历元
  (GPST)换算成 UTC。
- **退避/超时参数**(`rtkrcv_node` 的 `restart_delay_s`/`crash_loop_life_s`/
  `max_restart_delay_s`(子进程重启退避)、`sol_initial_backoff_s`/
  `sol_max_backoff_s`/`sol_idle_timeout_s`(连接 `rtkrcv` 解算输出口的重连与
  空闲检测),以及 `rtcm_bridge` 每路流各自的 `initial_backoff_s`/
  `max_backoff_s`/`idle_timeout_s`)——这些是隧道内弱链路场景下运维最可能需要
  现场调整的旋钮:链路抖动大就适当放宽退避上限和空闲超时,链路稳定就可以收紧
  以更快发现故障。

  **`sol_idle_timeout_s` 不接受 0**:节点会在启动时拒绝这个值。0 会被底层
  `TcpStream` 当成"彻底关闭空闲检测"的哨兵值,而这条自愈能力
  ("`rtkrcv` 连接卡死了要能被自动发现并恢复")正是这个节点存在的核心意义,
  不允许通过配置在现场被悄悄关掉。

完整参数表见 `config/gnss_bringup.yaml` 里的注释——文件本身就是文档,每个参数
旁边都带着来源(哪个节点声明、默认值多少)。

`pos_writer` 是独立进程,不会自动读取 `rtkrcv_node` 的任何参数,即使参数名字
相同(比如 `leap_seconds`)也要在 `pos_writer` 自己的段里重复声明一遍——两边
分别改动、忘了同步是这类"看起来一份配置、实际两份"参数最容易踩的坑。

## `.pos` 输出:目录布局、轮转与沉默检测

`pos_writer` 把每一路 `gnss_msgs/RtkFix` 按 1 Hz(`period_s`,spec §5.3)抽稀,
写成 RTKLIB 兼容的 `.pos` 文本摘要,供人工检查或轻量级下游工具使用——它不是
全量数据,全量原始流的记录见下面「rosbag2 录制」一节。

**目录布局**:`<root>/YYYYMMDD/<source>.pos`。`<root>` 是 `pos_writer.root`
参数,`<source>` 是 `pos_writer.sources` 里声明的每一路名字(默认只有
`can`,见上面 `config/gnss_bringup.yaml` 里 `sources` 旁的注释),`YYYYMMDD`
按 **UTC** 自然日、在 **UTC 零点**换文件——依据的是**每条记录自己的时间戳**
(`gnss_time`,缺失时兜底用 `header.stamp`),而不是节点启动时刻或墙钟当前
时间。这个设计是有意的:**回放一份历史 bag 时,写出的 `.pos` 会落在数据本身
发生的那个日期目录下**,而不是落在回放这个动作发生的今天。

**`root` 不需要预先存在**:`gnss_core::PosWriter::open()` 在打开每一个
`.pos` 文件之前,会对它所在的目录(`<root>/YYYYMMDD/`)调用
`std::filesystem::create_directories()`——这会**连同 `root` 本身一起**递归
建出来,不存在只是"忽略已存在"这一种错误。真正需要满足的条件是:`root`
最近的一个**已经存在**的祖先目录必须允许本进程创建子目录(有写权限、没有
只读挂载、磁盘没满)。建不出目录、或者目录建出来了但文件打不开(权限/磁盘
问题),`open()` 都会返回 `false`,节点侧翻译成一条 ERROR(见下面沉默检测
一节的两种告警文案)。

**时间戳合理性闸门**:一条记录如果两个时间字段(`gnss_time`/`header.stamp`)都
缺失或异常(NaN/inf/接近 0/远超合理范围,合理范围是 2000-01-01 到 2100-01-01
之间),会被**直接丢弃、不写入、不建目录**,并打印一条节流过的 WARN(不刷屏,
附带累计丢弃计数),日志会区分具体原因(字段确实缺失,还是字段有值但不像真实
的 GNSS 时刻)。这是刻意的设计:不这样做的话,坏时间戳会让数据静默落进一个
`<root>/19700101/` 这种没人会想起来查的目录,比"丢弃并报警"更危险。

**重启/重放接续同一个文件时的去重(final-fix-wave 第 1 项,round 2 复盘后
改成精确成员判定)**:`.pos` 是 `calibrate_sigma_scale` 之类下游工具的量测
输入,每一行都会被当成一条独立的观测——bag 重放、进程重启后重新收到同一段
数据、或者不小心在同一台机器上起了两个 `pos_writer` 实例,都会让同一段时间
的记录被写进同一个文件两次,从而被下游静默双倍加权。现在打开一个已经存在
且非空的 `.pos` 文件时,会对每一行都按 `.pos` 自己的分辨率(毫秒)算出它的
时间键,记进一个集合——之后任何一条记录,只要它渲染出的毫秒键已经在这个
集合里,就判定为与已有内容重复,**静默跳过、不落盘**(并立刻把新写的键也
加入集合,因此同一次运行内两次传入完全相同的 stamp 也会被挡住),同时按
`kDroppedBadStamp` 同样的方式打一条节流过的 WARN,附带"这段时间一共跳过了
多少条"的累计计数,而不是为区间里的每一条都打一行。这不是错误,而是重放/
重启接续同一份数据时的正常现象——而且现在是精确判定,凡是被跳过的都真的是
文件里已经原样存在的同一条记录,"不是数据丢失"这句话是准确的。

这个规则**允许乱序追加**:重放一段填补更早时段空洞的 bag(该段的记录早于
文件里已有的最后一条)会被正常写入,而不是被当成"整体重叠"连同新数据一起
丢弃——这是早期"stamp 小于等于文件最后一条就判定重叠"那版实现的一个真实
bug,已经修掉。

**断电导致最后一行没写完整**:如果 `pos_writer` 在写一行的中途被杀掉/掉电,
文件最后一行可能没有换行符结尾。重新打开这个文件时会把这半行**原地截断
丢弃**(一条记录存在与否以它的完整换行行是否存在为准,残缺的半行可能凑巧
解析出一个看似合理但错误的值)。这半条记录是真正丢失的数据,节点会打印一条
不节流的 ERROR("已原地截掉这半行——这半条记录已经丢失"),与上面"不是数据
丢失"的去重 WARN 分开报告,每次打开文件至多报一次。

**每路独立的沉默检测**:每一路都有一个独立于消息到达的 wall-clock 定时器——
如果某一路持续 `silence_timeout_s` 秒(默认 10 秒)没有写出任何一条记录,会
打印一次 WARN,**按实际原因分成两种文案**(final-fix-wave 第 4 项;此前两者
共用同一句话,root 打不开时会把操作人员引导去查一个其实完全正常的话题链路):

- **消息在到达,但没能成功写出**(过去 `silence_timeout_s` 秒内确实收到过
  这一路的消息,只是文件一直没有增长):提示去检查输出路径(`pos_writer.root`
  对应的路径)是否可写、磁盘是否已满——典型原因是默认 `root` 指向的路径没有
  权限创建。正常情况下这句 WARN 之前会先看到一条 `open`/`write` 失败的
  ERROR;如果没看到,说明失败锁存期内又发生了同类失败,详情看最早那一条。
- **完全没有收到过任何消息**:提示检查 1)话题名是否与实际发布者匹配
  (`pos_writer` 订阅固定用 `<name>.topic` 参数指定的话题名,拼错/没配对就是
  持续沉默);2)订阅固定是 **reliable QoS**——如果对端发布者是
  **best_effort**,`reliable` 订阅根本收不到任何消息,现象和"驱动没启动"
  完全一样,同样表现为持续沉默,容易被误判成"对端没在发布"。

## rosbag2 录制:全量原始流

`.pos` 是给人看的 1 Hz 摘要,**全量原始数据的记录与回放直接交给 rosbag2 承担**
(spec §5.2)——四路数据(两路 `RawStream` 裸流、各路 `RtkFix`、`rtkrcv_node` 的
`~/stat`)全部上了总线之后,不需要额外的自定义落盘逻辑。

```bash
source install/setup.bash
bash src/glim_ext/gnss_bringup/scripts/record_gnss.sh
```

默认录制:`/gnss/rtcm_corrections`、`/gnss/raw_obs`、`/gnss_cgi610/rtk_fix`、
`/gnss_cgi610/rtk_fix_gpchc`、`/rtkrcv_node/rtk_fix`、`/rtkrcv_node/stat`
六路话题,输出到 `$HOME/gnss_bags/gnss_<时间戳>/`(按 `--max-bag-duration` 分卷,
默认 86400 秒即一天一卷——注意这是"从录制进程启动那一刻起满 N 秒就切卷",不是
像 `.pos` 那样按 UTC 自然日对齐,两者是不同的机制)。启动时会先检查每个话题是否
已经在总线上,**不存在只警告、不阻止启动**——录制一个当前还没有发布者的话题是
合法的,链路上各个节点完全可能按不同顺序、先后起来。输出根目录、话题清单、
分卷时长、存储后端均可用环境变量覆盖,默认值和用法写在脚本顶部的注释里
(`GNSS_BAG_ROOT`/`GNSS_BAG_TOPICS`/`GNSS_BAG_MAX_DURATION_S`/`GNSS_BAG_STORAGE`)。

**缺口(spec A6,留给轮 3,这里只记录不实现)**:**rosbag2 本身没有按保留天数
或磁盘水位自动清理旧 bag 的能力**——这个脚本只管"怎么录",不管"录多了怎么删"。
长期运行的部署必须由运维自行监控 `GNSS_BAG_ROOT` 所在磁盘的占用,并用外部定时
任务(cron/systemd timer 之类)手动清理超期的 bag,否则磁盘会被写满。旧 `.pos`
文件的 gzip 压缩同样不在本轮范围内(见文末「遗留」)。

## 已验证 / 未验证项

2026-09-14 在开发机上用 **RTKLIB-EX 2.5.1** 验证过。回归用例是 `test/test_rtkrcv_real_binary.cpp`
(未装 rtkrcv 或 libfaketime 时自动跳过)与 `test/test_rtkrcv_node_process.cpp`:

- 生成的 conf 键全部被 2.5.1 识别并生效(用 rtkrcv 控制台 `option` 逐项核对过)
- `-s -nc -r 2 -o <conf>` 能无终端常驻,SIGTERM 下正常退出
- 两路上行按字节原样送达(corrections → `inpstr2`,raw_obs → `inpstr1`),节点连得上 `sol_port`,
  `.stat` 文件名为 `rtkrcv_%Y%m%d%h%M.stat`
- 真实双站 RTCM3 回放(RTKLIB 自带 2005 年 GSI 两站 RINEX 转换而来,基线约 3.3 km):
  `RtkFix` 与 rtkrcv 原始解算行逐字段一致(经纬高、质量、NEU→ENU 标准差、卫星数、龄期),
  `pos_writer` 按 UTC 日轮转写出 `.pos`

仍未验证:

- **固定率**:回放数据只得到浮点解(转换成 RTCM 时丢了锁定信息,原始 RINEX 后处理可以固定),
  固定率只能用现场数据判断
- 现场板卡的原始观测格式,以及平台差分流是否带 1005/1006(`base_pos_type: rtcm` 的前提)
- 现场端点与连接方向(见「现场待确认」)
- 长时间运行、真实丢星与 AR 状态切换下的解算行

## 已知问题

- **`rtkrcv_node` 的 `args` 参数不能在 YAML 里写成空列表**——`args: []` 会让
  ROS2 的 YAML 参数加载器因为无法从空列表推断元素类型而在节点启动时抛
  `InvalidParameterValueException`,直接 `std::terminate()`(已实测复现)。
  当前的规避方式是 `config/gnss_bringup.yaml` 里干脆不写这个 key(节点侧
  `args` 参数本身默认值就是空列表,不写等价于"不传额外参数")。
  这只是绕开了症状,没有解决根因:任何人往这份 YAML 里加一行 `args: []`
  仍然会复现同样的崩溃,而且报错信息不会指向这份 README。正确的修复是在
  `rtkrcv_node.cpp` 声明该参数时,显式指定 `rcl_interfaces::msg::ParameterDescriptor`
  把类型固定为字符串数组(而不是依赖默认值推断类型),或者干脆换成一个用分隔符
  拼接的字符串参数(例如 `extra_args: "-s"`,按空格切分)。这不属于 Task 8 的
  修改范围(会改到 `rtkrcv_node.cpp`),记在这里留给后续任务处理。

## 不装 RTKLIB 也能跑起来:用 `test/fake_rtkrcv.sh`

`test/fake_rtkrcv.sh` 是一个不需要真实 RTKLIB 就能跑的替身二进制,足以练习整条
监管/转发管道(启动、conf 生成、TCP 转发、SIGTERM 收尾)。`launch.py` 里
`binary` 是写在 YAML 里的节点参数,不是 launch 参数,不能用
`ros2 launch ... key:=value` 直接覆盖;两种可行方式:

复制一份 `config/gnss_bringup.yaml`,把 `rtkrcv_node.binary` 改成
`fake_rtkrcv.sh` 的绝对路径、`args` 改成 `["live"]`,再用
`params_file:=<改过的路径>` 整份替换:

```bash
ros2 launch gnss_bringup gnss_bringup.launch.py params_file:=/path/to/fake.yaml
```

或者不经过 launch 文件,直接单独跑 `rtkrcv_node` 验证监管/转发逻辑:

```bash
ros2 run gnss_bringup rtkrcv_node --ros-args \
  -p binary:="$(pwd)/src/gnss_bringup/test/fake_rtkrcv.sh" -p "args:=['live']" \
  -p run_dir:=/tmp/rtkrcv_smoke_run
```

或者只起 `rtcm_bridge`(完全不涉及 rtkrcv,不需要任何替身):

```bash
ros2 launch gnss_bringup gnss_bringup.launch.py enable_rtkrcv:=false
```

配合 `nc` 手动灌一段字节验证桥接:

```bash
# rtcm_corrections 默认 listen=false,即 rtcm_bridge 主动连 127.0.0.1:15031;
# 用 nc -l 在该端口起一个假的差分源
nc -l 15031 < some_bytes.bin
ros2 topic echo /gnss/rtcm_corrections
```
