#!/usr/bin/env bash
# run_field_integration.sh —— 现场数据整链路回放一遍(Task 3)
#
# 用法:
#   run_field_integration.sh [--force] <A|B|dry|dryB> <seg> [duration_s]
#
#   --force  运行目录 run_<mode> 已存在且非空时清空后重跑;不给则拒绝(退出码 2),以免覆盖已有证据
#
#   A     原样:差分 = <seg>/gnss/base.rtcm3,观测 = <seg>/raw/cgi610.dat,整段 1×
#   B     补星历:同 A,另把 <seg>/gnss/rover.nav 编码成 RTCM3 星历电文,注入差分流
#         (第一块之前一次,之后每 30 数据秒一次)
#   dry   试跑:A 的输入,只放前 duration_s 数据秒(默认 90)
#   dryB  试跑:B 的输入,只放前 duration_s 数据秒(默认 90)
#
# 环境变量(可选):
#   GNSS_FIELD_INTEG_DIR  产物根目录 INTEG(须已存在,且路径里有 integration_* 这一级)。缺省:
#                         <seg>/integration_20260916 存在就用它(兼容 2026-09-16 那一轮),
#                         否则 <seg>/integration_<今天 YYYYMMDD>
#   GLIM_WS_INSTALL       缺省 /home/steve/glim_ws/install(提供 gnss_bringup)
#   DRIVER_WS_INSTALL     缺省 /home/steve/driver_ws/install(提供 gnss_chcnav)
#   RTKLIB_SRC            RTKLIB-EX 源码树(B/dryB 编译 rnx_nav_to_rtcm.c 用)
#   RTKRCV_TRACE_LEVEL    1-5 时给 rtkrcv 加 -t <level>
#
# 产物全部在 RUN=$INTEG/run_<mode>/(非空时须 --force 才清空重跑,只清这一个目录):
#   params.yaml              由 gnss_bringup.yaml 改出来的参数覆盖
#   can_with_sigma.log       补了全零 σ 帧(0x326/0x328/0x32B)的 CAN 日志副本
#   nav.rtcm3                (B/dryB)rover.nav 编码出的星历电文
#   rtkrcv/                  rtkrcv 运行目录(rtkrcv.conf、rtkrcv_*.stat)
#   pos/YYYYMMDD/*.pos       pos_writer 输出(can.pos、rtkrcv.pos)
#   diag/                    gnss_diag_node 输出(YYYYMMDD/events.log、base.pos、base_baseline)
#   bags/gnss_*/             rosbag2 录包
#   logs/*.log               各进程 stdout/stderr;logs/run.log 是本脚本自己的记录
#   roslog/                  ROS_LOG_DIR
#
# 前置条件见同目录 README.md(vcan0 已建好且 UP、can-utils、RTKLIB-EX 2.5.1 的 rtkrcv 与
# librtklib、glim_ws 与 driver_ws 已编译)。
#
# 退出码:field_replay.py 的退出码(非 0 时);回放成功但收尾后仍有残留进程时为 6;
# 前置检查失败为 2。

set -o pipefail
# 注意:不开 set -u —— /opt/ros/humble/setup.bash 引用未定义变量

HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

# 进程残留检查。控制者给的检查式是
#   pgrep -af "rtkrcv|rtcm_bridge|pos_writer|gnss_diag|gnss_chcnav|canplayer|ros2 bag|field_replay"
# 但它会命中任何命令行里恰好带这些词的无关进程(本脚本自身的路径就含 field_replay,
# 别的终端里 grep/tail 这些日志的 shell 也会中招,2026-09-16 dryB 试跑就误报过一次)。
# 这里改成锚定在程序名上:命令行第一个词(或 python 解释器后的脚本/ros2 子命令)是这些程序才算。
# Fix round 1:原模式漏掉 rtkrcv_node(`rtkrcv( |$)` 不认 rtkrcv_node)、gnss_cleanup_node、
# 不带解释器前缀的 `ros2 bag record`、python3.10 这类解释器名。样例见 tests/test_proc_pattern.py。
# 最终修复轮:具体程序名(rtkrcv、各节点、canplayer、field_replay.py)不分 ROS domain 一律算——别的会话里的
# 同名进程也会抢 vcan0 / 本机端口,照样要先处理;通用的 `ros2 bag|launch|run` 只算 ROS_DOMAIN_ID 等于本脚本
# 所用 domain 的(读 /proc/<pid>/environ,读不到的算别人的),别的终端里无关的 ros2 launch 不再误报。
RUN_DOMAIN_ID=66
PROC_PAT='^(\S*/)?(rtkrcv|rtkrcv_node|rtcm_bridge|pos_writer|gnss_diag_node|gnss_cleanup_node|gnss_chcnav_can|canplayer)( |$)'
PROC_PAT+='|^(\S*/)?python[0-9.]*( -u)? (\S*/)?field_replay\.py( |$)'
ROS2_CLI_PAT='^(\S*/)?python[0-9.]*( -u)? (\S*/)?ros2 (bag|launch|run)( |$)'
ROS2_CLI_PAT+='|^(\S*/)?ros2 (bag|launch|run)( |$)'
in_run_domain() {  # in_run_domain <pid>:该进程启动时的环境里 ROS_DOMAIN_ID 等于 RUN_DOMAIN_ID
  { tr '\0' '\n' < "/proc/$1/environ"; } 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$RUN_DOMAIN_ID"
}
leftovers() {
  pgrep -af "$PROC_PAT" || true
  local pid args
  while read -r pid args; do
    [[ -n "$pid" ]] && in_run_domain "$pid" && printf '%s %s\n' "$pid" "$args"
  done < <(pgrep -af "$ROS2_CLI_PAT" || true)
  return 0
}

# ERE 转义(路径里的 . + ( ) 等按字面匹配)
ere_escape() { printf '%s' "$1" | sed 's/[][\.*^$+?(){}|]/\\&/g'; }
# 收尾兜底用:只认 rtkrcv 程序本身、且 -o 参数正好是本次运行的 conf(rtkrcv_node 的启动参数是
# `rtkrcv -s -nc -r 2 -o <conf> [args...]`);tail/vim 这个 conf 的进程、别的运行目录的 rtkrcv 都不算
rtkrcv_conf_pat() { printf '^(\\S*/)?rtkrcv( .*)? -o %s( |$)' "$(ere_escape "$1/rtkrcv/rtkrcv.conf")"; }

# 自检用(不需要 ROS),tests/test_proc_pattern.py 用:
#   --print-proc-pat / --print-ros2-cli-pat  打印模式,拿它对样例命令行跑 grep -E
#   --print-rtkrcv-pat <run>                 打印收尾兜底用的 rtkrcv 模式
#   --list-leftovers                         打印此刻的残留检查结果(验证 domain 过滤)
case "${1:-}" in
  --print-proc-pat) printf '%s\n' "$PROC_PAT"; exit 0 ;;
  --print-ros2-cli-pat) printf '%s\n' "$ROS2_CLI_PAT"; exit 0 ;;
  --print-rtkrcv-pat) rtkrcv_conf_pat "${2:?}"; echo; exit 0 ;;
  --list-leftovers) leftovers; exit 0 ;;
esac

FORCE=0
POS_ARGS=()
for a in "$@"; do
  case "$a" in
    --force) FORCE=1 ;;
    -*) echo "未知选项: $a" >&2; exit 2 ;;
    *) POS_ARGS+=("$a") ;;
  esac
done
MODE="${POS_ARGS[0]:-}"
SEG="${POS_ARGS[1]:-}"
DUR_ARG="${POS_ARGS[2]:-}"

usage() { sed -n '2,22p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 2; }
[[ ${#POS_ARGS[@]} -le 3 ]] || usage
[[ -n "$MODE" && -n "$SEG" ]] || usage

case "$MODE" in
  A)    NAV=0; DUR="" ;;
  B)    NAV=1; DUR="" ;;
  dry)  NAV=0; DUR="${DUR_ARG:-90}" ;;
  dryB) NAV=1; DUR="${DUR_ARG:-90}" ;;
  *) usage ;;
esac
if [[ ( "$MODE" == A || "$MODE" == B ) && -n "$DUR_ARG" ]]; then
  echo "A/B 是整段回放,不接受 duration_s(试跑用 dry/dryB)" >&2; exit 2
fi
if [[ -n "$DUR" && ! "$DUR" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "duration_s 必须是正数: $DUR" >&2; exit 2
fi

SEG="$(readlink -f "$SEG")"
RTCM="$SEG/gnss/base.rtcm3"
OBS="$SEG/raw/cgi610.dat"
CAN_SRC="$SEG/raw/can7.candump.log"
RNX_NAV="$SEG/gnss/rover.nav"
if [[ -n "${GNSS_FIELD_INTEG_DIR:-}" ]]; then
  INTEG="$GNSS_FIELD_INTEG_DIR"
elif [[ -d "$SEG/integration_20260916" ]]; then
  INTEG="$SEG/integration_20260916"
else
  INTEG="$SEG/integration_$(date +%Y%m%d)"
fi
GLIM_WS_INSTALL="${GLIM_WS_INSTALL:-/home/steve/glim_ws/install}"
DRIVER_WS_INSTALL="${DRIVER_WS_INSTALL:-/home/steve/driver_ws/install}"

for f in "$RTCM" "$OBS" "$CAN_SRC"; do
  [[ -r "$f" ]] || { echo "缺输入文件: $f" >&2; exit 2; }
done
if [[ $NAV == 1 && ! -r "$RNX_NAV" ]]; then echo "缺输入文件: $RNX_NAV" >&2; exit 2; fi
[[ -d "$INTEG" ]] || { echo "缺目录 $INTEG(数据段只读,产物只写这里,需先建好;或用 GNSS_FIELD_INTEG_DIR 指定)" >&2; exit 2; }
INTEG="$(readlink -f "$INTEG")"
RUN="$INTEG/run_$MODE"

# 名字守卫(任何清空之前):只清 <…/integration_*/…>/run_<A|B|dry|dryB>,INTEG 不能就是数据段目录本身
case "$RUN" in
  */integration_*/run_A|*/integration_*/run_B|*/integration_*/run_dry|*/integration_*/run_dryB) ;;
  *) echo "RUN=$RUN 不是预期的运行目录(路径里要有 integration_* 这一级),拒绝使用" >&2; exit 2 ;;
esac
[[ "$INTEG" != "$SEG" && "$INTEG" != / ]] || { echo "INTEG=$INTEG 不能是数据段目录或 /" >&2; exit 2; }
# 不覆盖已有证据:运行目录非空时要 --force 才清空
if [[ -d "$RUN" && -n "$(find "$RUN" -mindepth 1 -maxdepth 1 -print -quit)" && $FORCE != 1 ]]; then
  echo "运行目录 $RUN 已存在且非空,拒绝覆盖(里面可能是要保留的结果)。" >&2
  echo "换一个产物根目录(GNSS_FIELD_INTEG_DIR=<新目录>),或确认可以丢弃后加 --force 重跑。" >&2
  exit 2
fi

# ---------- 前置检查 ----------
if ip link show vcan0 2>/dev/null | grep -q "UP"; then :; else
  echo "vcan0 不存在或未 UP(需要维护者用 sudo 建:见 README)" >&2; exit 2
fi
for b in canplayer rtkrcv gcc python3; do
  command -v "$b" >/dev/null || { echo "找不到 $b" >&2; exit 2; }
done
if [[ -n "$(leftovers)" ]]; then
  echo "已有相关进程在跑,先处理掉再回放(避免串扰):" >&2; leftovers >&2; exit 2
fi

# ---------- 1. 运行目录与环境 ----------
case "$RUN" in   # 与上面的名字守卫相同,紧挨着清空再查一次
  */integration_*/run_A|*/integration_*/run_B|*/integration_*/run_dry|*/integration_*/run_dryB) ;;
  *) echo "内部错误:RUN=$RUN 不是预期的运行目录,拒绝清空" >&2; exit 2 ;;
esac
mkdir -p "$RUN"
find "$RUN" -mindepth 1 -delete
mkdir -p "$RUN/logs" "$RUN/roslog" "$RUN/rtkrcv" "$RUN/pos" "$RUN/diag" "$RUN/bags"
# tee 忽略 SIGINT:终端 Ctrl-C 会发给整个前台进程组,tee 先死的话后面收尾的输出会让脚本吃 SIGPIPE
exec > >(trap '' INT; exec tee -a "$RUN/logs/run.log") 2>&1

say() { echo "[run_field_integration $(date +%H:%M:%S)] $*"; }
say "mode=$MODE seg=$SEG run=$RUN nav=$NAV duration=${DUR:-整段} force=$FORCE"
say "GLIM_WS_INSTALL=$GLIM_WS_INSTALL DRIVER_WS_INSTALL=$DRIVER_WS_INSTALL"

export ROS_DOMAIN_ID=$RUN_DOMAIN_ID
export ROS_LOG_DIR="$RUN/roslog"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$GLIM_WS_INSTALL/setup.bash" || { say "source $GLIM_WS_INSTALL/setup.bash 失败"; exit 2; }
# driver_ws 放最后:提供 gnss_chcnav(Task 1 核对过两份 gnss_msgs 同源,都带 ratio)
# shellcheck disable=SC1091
source "$DRIVER_WS_INSTALL/setup.bash" || { say "source $DRIVER_WS_INSTALL/setup.bash 失败"; exit 2; }
command -v ros2 >/dev/null || { say "source 后仍找不到 ros2"; exit 2; }
BRINGUP_PREFIX="$(ros2 pkg prefix gnss_bringup)" || { say "找不到 gnss_bringup 包"; exit 2; }
ros2 pkg prefix gnss_chcnav >/dev/null || { say "找不到 gnss_chcnav 包(driver_ws 未编译?)"; exit 2; }

# ---------- 2. 输入准备:CAN σ 补帧、星历电文 ----------
python3 "$HERE/can_sigma_patch.py" "$CAN_SRC" "$RUN/can_with_sigma.log" || { say "CAN 补帧失败"; exit 2; }

NAV_RTCM=""
if [[ $NAV == 1 ]]; then
  RTKLIB_SRC="${RTKLIB_SRC:-/home/steve/Documents/GitHub/gnss-alg/RTKLIB-2.5.1}"
  mkdir -p "$RUN/tools"
  # -D 宏必须与 /usr/local/lib/librtklib.so 构建时一致(见 rnx_nav_to_rtcm.c 文件头)
  if ! gcc -O2 -DDLL -DENACMP -DENAGAL -DENAGLO -DENAIRN -DENAQZS -DNEXOBS=3 -DNFREQ=3 -DTRACE \
       -I"$RTKLIB_SRC/src" -o "$RUN/tools/rnx_nav_to_rtcm" "$HERE/rnx_nav_to_rtcm.c" \
       -L/usr/local/lib -lrtklib -lm -lpthread > "$RUN/logs/rnx_nav_to_rtcm_build.log" 2>&1; then
    say "编译 rnx_nav_to_rtcm 失败,见 logs/rnx_nav_to_rtcm_build.log"; exit 2
  fi
  LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}" "$RUN/tools/rnx_nav_to_rtcm" "$RNX_NAV" "$RUN/nav.rtcm3" \
    || { say "rnx_nav_to_rtcm 失败"; exit 2; }
  NAV_RTCM="$RUN/nav.rtcm3"
  if [[ -r "$INTEG/logs/rover_eph.rtcm3" ]]; then
    if cmp -s "$NAV_RTCM" "$INTEG/logs/rover_eph.rtcm3"; then
      say "nav.rtcm3 与 logs/rover_eph.rtcm3 逐字节相同"
    else
      say "注意:nav.rtcm3 与 logs/rover_eph.rtcm3 不同(rnx_nav_to_rtcm.c 可能已修改),按本次生成的发"
    fi
  fi
fi

# ---------- 3. 参数覆盖 ----------
# 排查用:RTKRCV_TRACE_LEVEL=<1-5> 时给 rtkrcv 加 -t <level>,trace 文件落在 rtkrcv/ 下(level 3 以上很大)
python3 - "$BRINGUP_PREFIX/share/gnss_bringup/config/gnss_bringup.yaml" "$RUN/params.yaml" "$RUN" "${RTKRCV_TRACE_LEVEL:-}" <<'PY' || { say "生成 params.yaml 失败"; exit 2; }
import sys
import yaml
src, dst, run, trace_level = sys.argv[1:5]
with open(src) as f:
    d = yaml.safe_load(f)
r = d["rtkrcv_node"]["ros__parameters"]
r["obs_format"] = "oem4"
r["run_dir"] = run + "/rtkrcv"
if trace_level:
    r["args"] = ["-t", trace_level]
p = d["pos_writer"]["ros__parameters"]
p["root"] = run + "/pos"
p["sources"] = ["can", "rtkrcv"]
p["rtkrcv"] = {"topic": "/rtkrcv_node/rtk_fix"}
g = d["gnss_diag"]["ros__parameters"]
g["root"] = run + "/diag"
g["startup_grace_s"] = 60.0
with open(dst, "w") as f:
    f.write("# 由 run_field_integration.sh 从 %s 生成,勿手改\n" % src)
    yaml.safe_dump(d, f, sort_keys=False, allow_unicode=True)
PY

# ---------- 4. 后台起全部节点(各自 setsid 成独立进程组,收尾时整组发 SIGINT) ----------
declare -A PIDS=()
start_bg() {  # start_bg <name> <cmd...>
  # setsid 在调用者恰好是进程组组长时(比如开了作业控制的 shell)会先 fork 再 setsid,$! 就不是
  # 真正的进程组号了;所以让新会话里的那个进程自己把 PID(= 进程组号)写到文件里再 exec
  local name=$1; shift
  local pidfile="$RUN/logs/.$name.pid"
  rm -f "$pidfile"
  setsid bash -c 'echo $$ > "$0"; exec "$@"' "$pidfile" "$@" > "$RUN/logs/$name.log" 2>&1 < /dev/null &
  local i
  for ((i = 0; i < 50; i++)); do [[ -s "$pidfile" ]] && break; sleep 0.1; done
  PIDS[$name]="$(cat "$pidfile" 2>/dev/null)"
  rm -f "$pidfile"
  if [[ -z "${PIDS[$name]}" ]]; then say "启动 $name 失败(拿不到 PID): $*"; return 1; fi
  say "已启动 $name pid=${PIDS[$name]}: $*"
}

group_alive() {  # 进程组里还有非僵尸进程(ros2 CLI 守护进程不算,见下面 ros2 daemon start 的说明)
  ps -e -o pgid=,stat=,args= | awk -v g="$1" '$1==g && $2 !~ /^Z/ && !/ros2-daemon/ {found=1} END {exit !found}'
}

FORCED=()
stop_group() {  # stop_group <name>:SIGINT 整个进程组,最多等 20 s,超时 SIGKILL
  local name=$1 pid=${PIDS[$1]:-}
  [[ -n "$pid" ]] || return 0
  if ! group_alive "$pid"; then
    say "$name 已先行退出(见 logs/$name.log)"; return 0
  fi
  kill -INT -- "-$pid" 2>/dev/null
  local i
  for ((i = 0; i < 200; i++)); do
    group_alive "$pid" || break
    sleep 0.1
  done
  if group_alive "$pid"; then
    say "$name 20 s 内未退出,SIGKILL 整个进程组"
    ps -e -o pid=,pgid=,args= | awk -v g="$pid" '$2==g' | sed 's/^/    /'
    kill -KILL -- "-$pid" 2>/dev/null
    FORCED+=("$name")
    sleep 0.5
  fi
  say "$name 已停止(用时 $(awk -v n="$i" 'BEGIN{printf "%.1f", n/10}') s)"
}

CLEANED=0
cleanup() {
  [[ $CLEANED == 1 ]] && return
  CLEANED=1
  say "收尾:依次停止 录包、诊断节点、驱动、launch"
  stop_group record
  stop_group gnss_diag
  stop_group gnss_chcnav
  stop_group bringup
  # rtkrcv 在自己的会话里(rtkrcv_node 的 ProcessSupervisor 用 setsid),launch 退出时由
  # rtkrcv_node 负责停;万一节点被 SIGKILL,这里兜底——只认 rtkrcv 程序且 -o 是本次 conf(见 rtkrcv_conf_pat)
  local rtk_pat; rtk_pat="$(rtkrcv_conf_pat "$RUN")"
  if pgrep -f "$rtk_pat" >/dev/null; then
    say "rtkrcv 仍在(节点未能停掉它),SIGTERM:"; pgrep -af "$rtk_pat"
    pkill -TERM -f "$rtk_pat"; sleep 3
    pkill -KILL -f "$rtk_pat" 2>/dev/null && FORCED+=("rtkrcv")
  fi
  ros2 daemon stop > /dev/null 2>&1 || true
}
on_signal() {
  say "收到中断信号"
  cleanup
  sleep 1
  local left; left="$(leftovers)"
  if [[ -n "$left" ]]; then say "残留进程:"; echo "$left"; else say "残留进程检查:无"; fi
  exit 130
}
trap on_signal INT TERM

# 先在进程组外起好 ros2 CLI 守护进程:否则 record_gnss.sh 里的 `ros2 topic list` 会把它拉起在录包的
# 进程组里,收尾时整组等不到退出(2026-09-16 第一次试跑:录包 0.03 s 就停了,守护进程拖满 20 s 被 SIGKILL)
ros2 daemon start > /dev/null 2>&1 || true

start_bg bringup ros2 launch gnss_bringup gnss_bringup.launch.py \
  params_file:="$RUN/params.yaml" enable_rtkrcv:=true enable_cleanup:=false enable_diag:=false
# 诊断节点要 sim time,launch 不转发 use_sim_time,单独起
start_bg gnss_diag ros2 run gnss_bringup gnss_diag_node --ros-args \
  --params-file "$RUN/params.yaml" -p use_sim_time:=true -p solver_enabled:=true
start_bg gnss_chcnav ros2 run gnss_chcnav gnss_chcnav_can --ros-args \
  -r __node:=gnss_cgi610 -p can_device:=vcan0 -p timestamp_source:=gps
BAG_TOPICS="/gnss/rtcm_corrections /gnss/raw_obs /gnss_cgi610/rtk_fix /gnss_cgi610/rtk_fix_gpchc /rtkrcv_node/rtk_fix /rtkrcv_node/stat /gnss/diagnostics /rtkrcv_node/diagnostics /clock"
start_bg record env GNSS_BAG_ROOT="$RUN/bags" GNSS_BAG_TOPICS="$BAG_TOPICS" \
  bash "$BRINGUP_PREFIX/lib/gnss_bringup/record_gnss.sh"

# ---------- 5. 回放 ----------
sleep 5
for name in bringup gnss_diag gnss_chcnav record; do
  group_alive "${PIDS[$name]}" || say "警告:$name 在回放开始前就退出了,见 logs/$name.log"
done
# rtkrcv_node 对没有 rtkrcv 连入时收到的字节直接丢弃:等 rtkrcv 连上它的本机端口再开始
# (conf 写了 misc-timeout=0,连上之后不会空闲断开;见 replay_plan.py)
RTKRCV_PORTS="$(python3 -c 'import sys, yaml; p = yaml.safe_load(open(sys.argv[1]))["rtkrcv_node"]["ros__parameters"]; print("%d,%d" % (p["corr_port"], p["obs_port"]))' "$RUN/params.yaml")"
REPLAY_ARGS=(--rtcm "$RTCM" --obs "$OBS" --can-log "$RUN/can_with_sigma.log" --wait-connected-ports "$RTKRCV_PORTS")
[[ -n "$NAV_RTCM" ]] && REPLAY_ARGS+=(--nav-rtcm "$NAV_RTCM")
[[ -n "$DUR" ]] && REPLAY_ARGS+=(--duration-s "$DUR")
say "回放: field_replay.py ${REPLAY_ARGS[*]}"
python3 -u "$HERE/field_replay.py" "${REPLAY_ARGS[@]}" 2>&1 | (trap '' INT; exec tee "$RUN/logs/field_replay.log")
REPLAY_RC=${PIPESTATUS[0]}
say "field_replay.py 退出码 $REPLAY_RC"

# ---------- 6. 收尾 ----------
cleanup
trap - INT TERM
sleep 1
LEFT="$(leftovers)"
if [[ -n "$LEFT" ]]; then
  say "残留进程:"; echo "$LEFT"
else
  say "残留进程检查(pgrep -af '$PROC_PAT';ros2 bag/launch/run 只算 ROS_DOMAIN_ID=$RUN_DOMAIN_ID 的):无"
fi
[[ ${#FORCED[@]} -gt 0 ]] && say "被强制 SIGKILL 的:${FORCED[*]}"

# ---------- 7. 汇总 ----------
say "产物清单:"
find "$RUN" -path "$RUN/roslog" -prune -o -type f -printf '%10s  %P\n' | sort -k2 | sed 's/^/    /'
for bag in "$RUN"/bags/gnss_*; do
  [[ -d "$bag" ]] || continue
  say "ros2 bag info $bag"
  ros2 bag info "$bag" 2>&1 | sed 's/^/    /'
done
ros2 daemon stop > /dev/null 2>&1 || true
for lf in "$RUN"/logs/*.log; do
  [[ "$(basename "$lf")" == run.log ]] && continue
  say "---- $(basename "$lf") 尾部 ----"
  tail -n 15 "$lf" | sed 's/^/    /'
done

if [[ $REPLAY_RC -ne 0 ]]; then exit "$REPLAY_RC"; fi
if [[ -n "$LEFT" ]]; then exit 6; fi
exit 0
