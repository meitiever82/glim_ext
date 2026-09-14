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

**这是本分支目前最大的残留风险,不是"锦上添花"的建议。**

`ProcessSupervisor` 用 `setsid()` 把 `rtkrcv` 放进独立进程组,这是为了让
`stop()` 能对整组发信号收尾派生出来的孙进程——这个决定本身是对的。但它的
代价是:`rtkrcv` 子进程只在父进程走**正常的 `stop()`/析构路径**时才会被
一起收尾;父进程如果是被 `kill -9`(或者任何不给它机会跑完析构函数的方式,
比如 OOM killer)杀掉的,`rtkrcv` 子进程会被 init 收养、继续独立运行,
变成一个孤儿进程,而 `rtkrcv_node` 完全不知道这件事。

已实测复现的后果——而且是那种**看起来一切正常、实则数据完全错误**的最坏
情况:

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

**现在的规避方式(必须遵守,直到有代码层面的修复)**:

- `rtkrcv_node` 必须由 systemd 管理,并且 unit 文件里配置基于 cgroup 的
  `KillMode=control-group`(而不是默认的 `KillMode=process`)——这样
  systemd 停止/重启这个服务时,会对整个 cgroup(包含孤儿 `rtkrcv`)发信号,
  而不只是对 `rtkrcv_node` 自己的 pid。
- 如果确实发生了 `kill -9`(或者进程以任何方式被信号杀死而不是走
  `ros2 lifecycle`/`Ctrl-C` 之类的正常关闭路径),**在重启 `rtkrcv_node`
  之前必须先手动确认没有孤儿 `rtkrcv` 残留**(例如 `pgrep -a rtkrcv`),
  有的话先手动杀掉,再重启节点。跳过这一步、直接重启,现场表现可能是
  "看起来正常运行,却在发布过时/错误的解"这种最难排查的故障。

代码层面的正确修复需要一个设计决策(`prctl(PR_SET_PDEATHSIG)`——注意它是
线程级语义,`ProcessSupervisor` 的 fork() 发生在一个专用线程上,细节需要
仔细核对;或者在 `run_dir` 下维护一个 pidfile,由下一次启动时先检查再决定
要不要清理),刻意不在这一轮修复范围内,留给后续任务处理。

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
参数(必须非空、必须可写),`<source>` 是 `pos_writer.sources` 里声明的每一路
名字(如 `can`/`gpchc`/`rtkrcv`),`YYYYMMDD` 按 **UTC** 自然日、在 **UTC 零点**
换文件——依据的是**每条记录自己的时间戳**(`gnss_time`,缺失时兜底用
`header.stamp`),而不是节点启动时刻或墙钟当前时间。这个设计是有意的:**回放
一份历史 bag 时,写出的 `.pos` 会落在数据本身发生的那个日期目录下**,而不是
落在回放这个动作发生的今天。

**时间戳合理性闸门**:一条记录如果两个时间字段(`gnss_time`/`header.stamp`)都
缺失或异常(NaN/inf/接近 0/远超合理范围,合理范围是 2000-01-01 到 2100-01-01
之间),会被**直接丢弃、不写入、不建目录**,并打印一条节流过的 WARN(不刷屏,
附带累计丢弃计数),日志会区分具体原因(字段确实缺失,还是字段有值但不像真实
的 GNSS 时刻)。这是刻意的设计:不这样做的话,坏时间戳会让数据静默落进一个
`<root>/19700101/` 这种没人会想起来查的目录,比"丢弃并报警"更危险。

**每路独立的沉默检测**:每一路都有一个独立于消息到达的 wall-clock 定时器——
如果某一路持续 `silence_timeout_s` 秒(默认 10 秒)没有写出任何一条记录,会
打印一次 WARN。排查时先检查:1)话题名是否与实际发布者匹配(`pos_writer` 订阅
固定用 `<name>.topic` 参数指定的话题名,拼错/没配对就是持续沉默);2)订阅固定
是 **reliable QoS**——如果对端发布者是 **best_effort**,`reliable` 订阅根本收不
到任何消息,现象和"驱动没启动"完全一样,同样表现为持续沉默,容易被误判成
"对端没在发布"。

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

## 未验证项

本包开发机上**没有安装 RTKLIB**,以下事项只用 `test/fake_rtkrcv.sh`(一个模拟
`rtkrcv` 命令行行为但不真正解算的 shell 脚本)验证了进程监管、conf 生成与流
转发的"管道"是否正确,**没有用真实 rtkrcv 二进制验证过**:

- **conf 是否被真实 rtkrcv 接受**——`render_rtkrcv_conf()` 生成的键名
  (`pos1-posmode`、`pos1-elmask`、`pos2-armode`、`pos1-navsys`、`out-timesys`
  等)是否是真实 RTKLIB(demo5)认识的键名、是否还缺 Task 5 未覆盖到的必需
  字段,只能靠真机验证。
- **真实的 llh 解算内容与节奏**——`parse_llh_solution` 期望的列序是照文档和
  假设对齐的,真实 RTKLIB 在各种解质量(float/DGPS/单点)、丢星、AR 状态切换
  下实际吐出的行是否总能被正确解析,未验证。
- **真实 `$SAT`/`.stat` 文件格式与命名**——`-r 2` 参数下 rtkrcv 实际生成的
  文件名模式、是否会在长时间运行后滚动出多个文件、`plan_stat_tail()`
  按 mtime 取最新是否总能对上真实场景,未用真实二进制验证。
- **rtkrcv 对 SIGTERM 的真实响应**——`ProcessSupervisor` 的信号/超时升级逻辑
  本身已用假二进制验证过,但真实 rtkrcv 收到 SIGTERM 后是否会先 flush 完
  `.stat`/解算流再退出,未知。
- **崩溃循环退避在真实 conf 错误下的表现**——例如一个 rtkrcv 无法识别的 conf
  字段导致它秒退,`crash_loop_life_s` 退避是否如预期触发,未用真实二进制验证过
  (退避逻辑本身已用 `fake_rtkrcv.sh` 单测覆盖,这里只是没有用真实二进制复现过)。
- **多客户端/大流量下 `LocalReserver` 与 rtkrcv 的真实交互**——目前只验证了单个
  自制 TCP 客户端收到 `LocalReserver` 广播的字节;rtkrcv 作为 `tcpcli` 连入后的
  真实读取节奏、断线重连行为,未验证。
- **端到端**——`rtcm_bridge` → `rtkrcv_node` → `~/rtk_fix` 的全链路需要真实的
  差分流与观测流,现场设备到位前无法验证。

装上 demo5 版 RTKLIB 后,建议按上面七条逐一补验,而不是假定管道跑通了就等于
解算正确。

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
