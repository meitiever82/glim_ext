#!/usr/bin/env bash
# 让 colcon 能发现 gnss_core / gnss_bringup。
#
# 背景:colcon 一旦把某个目录识别为包就不再向下递归,而 src/glim_ext 本身是包,
# 所以 src/glim_ext/gnss_core、src/glim_ext/gnss_bringup 永远不会被发现 ——
# `colcon list` 里没有它们,`colcon build --packages-select gnss_core` 报
# "ignoring unknown package"。但 glim_ext/package.xml 声明了对它们的 <depend>。
#
# 解法:在 workspace 的 src/ 下建指向它们的符号链接(同 driver_ws/gnss_msgs 的做法)。
# workspace 根目录不是 git 仓库,该链接无法版本管理,故由本脚本幂等创建。
# 新机器 / 新克隆在 colcon build 之前必须先跑一次。
#
# 另外还要链接 gnss_msgs:它的源码正本在 finder_ros/drivers 里(不属于本仓),
# 链接进来之后 glim_ws 就自足了,构建前不必再 source driver_ws。
#
# !! 代价 !! gnss_msgs 因此会被编两遍(driver_ws 为了 gnss_chcnav 也在编它)。
# 改过 .msg 之后**两个 workspace 都要重建**,否则一边用新结构发、另一边用旧结构订,
# ROS2 Humble 下的表现是 DDS 静默不匹配:不报错、不警告,话题就是收不到。
# 轮 1 往 RtkFix 里加 gnss_time 时就正好是这个场景。
#
# 源码路径可用 GNSS_MSGS_SRC 覆盖,默认值见下。
#
# 用法: bash src/glim_ext/setup_workspace.sh
set -euo pipefail

GNSS_MSGS_SRC="${GNSS_MSGS_SRC:-$HOME/Documents/GitHub/ztpilot/finder_ros/drivers/gnss_msgs}"

glim_ext_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
src_dir="$(dirname "${glim_ext_dir}")"

for name in gnss_core gnss_bringup; do
  link="${src_dir}/${name}"
  target_rel="$(basename "${glim_ext_dir}")/${name}"
  if [[ ! -d "${glim_ext_dir}/${name}" ]]; then
    echo "error: ${glim_ext_dir}/${name} 不存在" >&2; exit 1
  fi
  if [[ -L "${link}" ]]; then
    current="$(readlink "${link}")"
    if [[ "${current}" == "${target_rel}" ]]; then echo "ok: ${link} -> ${current} (已存在)"; continue; fi
    echo "error: ${link} 已是指向 ${current} 的符号链接,与预期的 ${target_rel} 不符" >&2; exit 1
  fi
  if [[ -e "${link}" ]]; then echo "error: ${link} 已存在且不是符号链接,拒绝覆盖" >&2; exit 1; fi
  ln -s "${target_rel}" "${link}"
  echo "created: ${link} -> ${target_rel}"
done

# gnss_msgs:源码在本仓之外,用绝对路径链接
msgs_link="${src_dir}/gnss_msgs"
if [[ ! -d "${GNSS_MSGS_SRC}" ]]; then
  echo "error: gnss_msgs 源码不在 ${GNSS_MSGS_SRC};用 GNSS_MSGS_SRC=<path> 覆盖" >&2
  exit 1
fi
if [[ -L "${msgs_link}" ]]; then
  current="$(readlink "${msgs_link}")"
  if [[ "${current}" == "${GNSS_MSGS_SRC}" ]]; then
    echo "ok: ${msgs_link} -> ${current} (已存在)"
  else
    echo "error: ${msgs_link} 指向 ${current},与预期的 ${GNSS_MSGS_SRC} 不符;请人工确认后删除重建" >&2
    exit 1
  fi
elif [[ -e "${msgs_link}" ]]; then
  echo "error: ${msgs_link} 已存在且不是符号链接,拒绝覆盖" >&2
  exit 1
else
  ln -s "${GNSS_MSGS_SRC}" "${msgs_link}"
  echo "created: ${msgs_link} -> ${GNSS_MSGS_SRC}"
fi
