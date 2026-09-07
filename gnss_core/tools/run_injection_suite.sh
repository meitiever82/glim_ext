#!/usr/bin/env bash
# run_injection_suite.sh —— 合成 RTK 注入套件(spec §12.3,Task 13 Step 6)。**在 Orin 上跑。**
#
# 对同一段既有纯 LiDAR(+IMU)bag:
#   1. 无 GNSS 跑 glim_rosbag,dump 轨迹作真值 traj_ref.txt
#   2. synth_rtk_fix 由 traj_ref.txt 生成 clean / wrong_fix / stale / float / lever 五个 .pos,各转成合并 bag
#   3. 每个 bag 跑 glim_rosbag(rtk_global 启用),robust_kernel = huber / none 各一次,dump 轨迹
#   4. evo_ape(无则 traj_rmse.py)算每条轨迹相对 traj_ref.txt 的 RMSE,输出表格
#
# usage: run_injection_suite.sh <lidar_bag_dir> <work_dir> [--glim-config <dir>] [--dump-config <json>]
#        环境变量:GLIM_WS(默认 ~/glim_ws) DRIVER_WS(默认 ~/driver_ws) TOPIC(默认 /gnss_cgi610/rtk_fix)
#
# 前提(需按现场实际情况核对/修改的地方都标了 ### CHECK):
#   - glim_rosbag 能通过 config 目录里的 config_ros.json 读 bag 并 dump 轨迹到 dump_path/traj_imu.txt
#   - glim_ext 已以 ENABLE_RTK_GLOBAL=ON 构建,config_rtk_global.json 里 robust_kernel / T_imu_gnss 可被本脚本改写
#   - 本脚本用 python3 + json 原地改 config;每个 run 用独立的 config 拷贝,不污染原配置
set -euo pipefail

BAG=${1:?lidar bag dir}
WORK=${2:?work dir}
shift 2
GLIM_WS=${GLIM_WS:-$HOME/glim_ws}
DRIVER_WS=${DRIVER_WS:-$HOME/driver_ws}
TOPIC=${TOPIC:-/gnss_cgi610/rtk_fix}
GLIM_CONFIG_SRC=${GLIM_WS}/src/glim/config              ### CHECK: 现场 glim config 目录
while [[ $# -gt 0 ]]; do
  case "$1" in
    --glim-config) GLIM_CONFIG_SRC=$2; shift 2 ;;
    *) echo "unknown arg $1"; exit 2 ;;
  esac
done

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# 可执行:优先 install 目录,其次同目录(纯 CMake build)
SYNTH=""
for p in "$(command -v synth_rtk_fix || true)" "${GLIM_WS}/install/gnss_core/lib/gnss_core/synth_rtk_fix" "${HERE}/../build/synth_rtk_fix"; do
  [[ -n "$p" && -x "$p" ]] && SYNTH=$p && break
done
POS2BAG=${HERE}/pos_to_rtkfix_bag.py
RMSE_PY=${HERE}/traj_rmse.py
[[ -x "$SYNTH" ]] || { echo "synth_rtk_fix not found"; exit 1; }

# shellcheck disable=SC1090
source "${DRIVER_WS}/install/setup.bash"
# shellcheck disable=SC1090
source "${GLIM_WS}/install/setup.bash"

mkdir -p "$WORK"
cd "$WORK"

# ---------- 工具函数 ----------
# make_config <name> <robust_kernel> <lever_x> <lever_y> <lever_z> <enable_rtk 0|1> → 输出 config 目录
make_config() {
  local name=$1 kernel=$2 lx=$3 ly=$4 lz=$5 enable=$6
  local dir="$WORK/config_$name"
  rm -rf "$dir"; cp -r "$GLIM_CONFIG_SRC" "$dir"
  python3 - "$dir" "$kernel" "$lx" "$ly" "$lz" "$enable" "$WORK/dump_$name" "$TOPIC" <<'PY'
import json, sys, os
d, kernel, lx, ly, lz, enable, dump, topic = sys.argv[1:]
def load(p):
    with open(p) as f: return json.load(f)
def save(p, o):
    with open(p, "w") as f: json.dump(o, f, indent=2)
# --- config_ros.json:dump 路径 + 扩展模块列表 ---                       ### CHECK: 键名按现场 glim 版本
ros = load(os.path.join(d, "config_ros.json"))
g = ros.get("glim_ros", ros)
g["dump_path"] = dump
mods = [m for m in g.get("extension_modules", []) if "rtk_global" not in m and "gnss_global" not in m]
if enable == "1":
    mods.append("librtk_global.so")
g["extension_modules"] = mods
save(os.path.join(d, "config_ros.json"), ros)
# --- config_rtk_global.json:核 + 杆臂 + topic ---
p = os.path.join(d, "config_rtk_global.json")
if enable == "1":
    if not os.path.exists(p):
        # 从 glim_ext 默认配置拷一份                                      ### CHECK
        src = os.path.expanduser("~/glim_ws/src/glim_ext/config/config_rtk_global.json")
        save(p, load(src))
    c = load(p)
    r = c.get("rtk_global", c)
    r["robust_kernel"] = kernel
    r["T_imu_gnss"] = [float(lx), float(ly), float(lz)]
    r["rtk_fix_topic"] = topic
    r["stamp_source"] = "gnss_time"
    r["time_offset"] = 0.0
    save(p, c)
PY
  echo "$dir"
}

# run_glim <config_dir> → 轨迹文件路径
run_glim() {
  local cfg=$1
  local dump; dump=$(python3 -c "import json,sys;o=json.load(open('$cfg/config_ros.json'));print(o.get('glim_ros',o)['dump_path'])")
  rm -rf "$dump"; mkdir -p "$dump"
  ros2 run glim_ros glim_rosbag "$2" --ros-args -p config_path:="$cfg" -p auto_quit:=true >"$dump/glim.log" 2>&1   ### CHECK: 参数名
  local traj="$dump/traj_imu.txt"
  [[ -s "$traj" ]] || { echo "no traj at $traj (see $dump/glim.log)"; exit 1; }
  echo "$traj"
}

# rmse <ref> <est> [extra args] → 一行数字
rmse() {
  local ref=$1 est=$2; shift 2
  if command -v evo_ape >/dev/null 2>&1; then
    evo_ape tum "$ref" "$est" -a --t_max_diff 0.05 2>/dev/null | awk '/rmse/{r=$2} END{print "rmse=" r}'
  else
    python3 "$RMSE_PY" "$ref" "$est" --align "$@"
  fi
}

# ---------- 1. 真值:无 GNSS ----------
echo "== [1/4] reference run (no GNSS)"
CFG_REF=$(make_config ref none 0 0 0 0)
TRAJ_REF=$(run_glim "$CFG_REF" "$BAG")
cp "$TRAJ_REF" traj_ref.txt
T0=$(awk '!/^#/{print $1; exit}' traj_ref.txt)
T1=$(awk '!/^#/{t=$1} END{print t}' traj_ref.txt)
DUR=$(python3 -c "print($T1-$T0)")
echo "   traj_ref.txt: $(grep -vc '^#' traj_ref.txt) poses, ${DUR}s"

# ---------- 2. 合成五个 .pos + 合并 bag ----------
echo "== [2/4] synthesize"
# 时间段用相对起点秒数('+'),取轨迹中段
S1=$(python3 -c "print(round($DUR*0.3,1))"); S2=$(python3 -c "print(round($DUR*0.5,1))")
declare -A SYNTH_ARGS=(
  [clean]=""
  [wrong_fix]="--wrong-fix 0.05"
  [stale]="--stale +$S1 +$S2"
  [float]="--float +$S1 +$S2"
  [lever]="--lever 0 1 0"
)
for name in clean wrong_fix stale float lever; do
  # shellcheck disable=SC2086
  "$SYNTH" traj_ref.txt "synth_$name.pos" ${SYNTH_ARGS[$name]} --truth "truth_$name.pos" --seed 42
  rm -rf "bag_$name"
  python3 "$POS2BAG" "synth_$name.pos" --out "bag_$name" --topic "$TOPIC" --merge "$BAG"
done

# ---------- 3. 跑 glim × {huber, none} ----------
echo "== [3/4] glim runs"
declare -A RES
run_case() {   # run_case <name> <kernel> <lever_xyz…>
  local name=$1 kernel=$2 lx=$3 ly=$4 lz=$5
  local tag="${name}_${kernel}_${lx}${ly}${lz}"
  local cfg; cfg=$(make_config "$tag" "$kernel" "$lx" "$ly" "$lz" 1)
  local traj; traj=$(run_glim "$cfg" "bag_$name")
  cp "$traj" "traj_$tag.txt"
  local n_rej; n_rej=$(grep -o 'n_rejected=[0-9]*' "$(dirname "$traj")/glim.log" | tail -1 || true)
  RES[$tag]="$(rmse traj_ref.txt "traj_$tag.txt") $n_rej"
  echo "   $tag: ${RES[$tag]}"
}
for name in clean wrong_fix stale float; do
  run_case "$name" huber 0 0 0
  run_case "$name" none 0 0 0
done
# 杆臂用例:config 填 0 与填对 [0,1,0],只跑 huber
run_case lever huber 0 0 0
run_case lever huber 0 1 0

# ---------- 4. 汇总 ----------
echo "== [4/4] summary (RMSE vs traj_ref.txt, aligned)"
{
  printf '%-12s | %-40s | %-40s\n' injection huber none
  printf -- '-------------|------------------------------------------|------------------------------------------\n'
  for name in clean wrong_fix stale float; do
    printf '%-12s | %-40s | %-40s\n' "$name" "${RES[${name}_huber_000]}" "${RES[${name}_none_000]}"
  done
  printf '%-12s | %-40s | %-40s\n' "lever T=0" "${RES[lever_huber_000]}" "-"
  printf '%-12s | %-40s | %-40s\n' "lever T=010" "${RES[lever_huber_010]}" "-"
  echo
  echo "转弯段单独看(需自行填时间窗):python3 $RMSE_PY traj_ref.txt traj_lever_huber_000.txt --align --relative-time --from S --to E"
  echo
  echo "预期(spec §12.3):wrong_fix: huber≈clean / none 恶化;stale: n_rejected 增加、RMSE≈clean;"
  echo "               float: RMSE≈clean;lever: T=010 转弯段 RMSE 明显低于 T=0"
} | tee summary.txt
