#!/usr/bin/env bash
# run_field_integration.sh —— 现场数据整链路回放一遍(Task 3)
#
# 用法:
#   run_field_integration.sh <A|B|dry|dryB> <seg> [duration_s]
#
#   A     原样:差分 = <seg>/gnss/base.rtcm3,观测 = <seg>/raw/cgi610.dat,整段 1×
#   B     补星历:同 A,另把 <seg>/gnss/rover.nav 编码成 RTCM3 星历电文,注入差分流
#         (第一块之前一次,之后每 30 数据秒一次)
#   dry   试跑:A 的输入,只放前 duration_s 数据秒(默认 90)
#   dryB  试跑:B 的输入,只放前 duration_s 数据秒(默认 90)
#
# 产物全部在 RUN=<seg>/integration_20260916/run_<mode>/(每次运行先清空这个目录,只清这一个):
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
PROC_PAT='^(\S*/)?(rtkrcv|rtkrcv_node|rtcm_bridge|pos_writer|gnss_diag_node|gnss_cleanup_node|gnss_chcnav_can|canplayer)( |$)'
PROC_PAT+='|^(\S*/)?python[0-9.]*( -u)? (\S*/)?(ros2 (bag|launch|run)|field_replay\.py)( |$)'
PROC_PAT+='|^(\S*/)?ros2 (bag|launch|run)( |$)'
leftovers() { pgrep -af "$PROC_PAT" || true; }

# 自检用:打印上面的模式后退出(不需要 ROS),tests/test_proc_pattern.py 拿它对样例命令行跑 grep -E
if [[ "${1:-}" == --print-proc-pat ]]; then printf '%s\n' "$PROC_PAT"; exit 0; fi
MODE="${1:-}"
SEG="${2:-}"
DUR_ARG="${3:-}"

usage() { sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 2; }
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
INTEG="$SEG/integration_20260916"
RUN="$INTEG/run_$MODE"

for f in "$RTCM" "$OBS" "$CAN_SRC"; do
  [[ -r "$f" ]] || { echo "缺输入文件: $f" >&2; exit 2; }
done
if [[ $NAV == 1 && ! -r "$RNX_NAV" ]]; then echo "缺输入文件: $RNX_NAV" >&2; exit 2; fi
[[ -d "$INTEG" ]] || { echo "缺目录 $INTEG(数据段只读,产物只写这里,需先建好)" >&2; exit 2; }


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
case "$RUN" in
  */integration_20260916/run_A|*/integration_20260916/run_B|*/integration_20260916/run_dry|*/integration_20260916/run_dryB) ;;
  *) echo "内部错误:RUN=$RUN 不是预期的运行目录,拒绝清空" >&2; exit 2 ;;
esac
mkdir -p "$RUN"
find "$RUN" -mindepth 1 -delete
mkdir -p "$RUN/logs" "$RUN/roslog" "$RUN/rtkrcv" "$RUN/pos" "$RUN/diag" "$RUN/bags"
# tee 忽略 SIGINT:终端 Ctrl-C 会发给整个前台进程组,tee 先死的话后面收尾的输出会让脚本吃 SIGPIPE
exec > >(trap '' INT; exec tee -a "$RUN/logs/run.log") 2>&1

say() { echo "[run_field_integration $(date +%H:%M:%S)] $*"; }
say "mode=$MODE seg=$SEG run=$RUN nav=$NAV duration=${DUR:-整段}"

export ROS_DOMAIN_ID=66
export ROS_LOG_DIR="$RUN/roslog"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/steve/glim_ws/install/setup.bash
# driver_ws 放最后:提供 gnss_chcnav(Task 1 核对过两份 gnss_msgs 同源,都带 ratio)
# shellcheck disable=SC1091
source /home/steve/driver_ws/install/setup.bash
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
  # rtkrcv_node 负责停;万一节点被 SIGKILL,这里按本次 conf 路径兜底
  if pgrep -f "$RUN/rtkrcv/rtkrcv.conf" >/dev/null; then
    say "rtkrcv 仍在(节点未能停掉它),SIGTERM:"; pgrep -af "$RUN/rtkrcv/rtkrcv.conf"
    pkill -TERM -f "$RUN/rtkrcv/rtkrcv.conf"; sleep 3
    pkill -KILL -f "$RUN/rtkrcv/rtkrcv.conf" 2>/dev/null && FORCED+=("rtkrcv")
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
  say "残留进程检查(pgrep -af '$PROC_PAT'):无"
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
