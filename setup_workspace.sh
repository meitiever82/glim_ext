#!/usr/bin/env bash
# 让 colcon 能发现 gnss_core。
#
# 背景:colcon 一旦把某个目录识别为包就不再向下递归,而 src/glim_ext 本身是包,
# 所以 src/glim_ext/gnss_core 永远不会被发现 —— `colcon list` 里没有它,
# `colcon build --packages-select gnss_core` 报 "ignoring unknown package"。
# 但 glim_ext/package.xml 声明了 <depend>gnss_core</depend>。
#
# 解法:在 workspace 的 src/ 下建一个指向它的符号链接(同 driver_ws/gnss_msgs 的做法)。
# workspace 根目录不是 git 仓库,该链接无法版本管理,故由本脚本幂等创建。
# 新机器 / 新克隆在 colcon build 之前必须先跑一次。
#
# 用法: bash src/glim_ext/setup_workspace.sh
set -euo pipefail

glim_ext_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
src_dir="$(dirname "${glim_ext_dir}")"
link="${src_dir}/gnss_core"
target_rel="$(basename "${glim_ext_dir}")/gnss_core"

if [[ ! -d "${glim_ext_dir}/gnss_core" ]]; then
  echo "error: ${glim_ext_dir}/gnss_core 不存在" >&2
  exit 1
fi

if [[ -L "${link}" ]]; then
  current="$(readlink "${link}")"
  if [[ "${current}" == "${target_rel}" ]]; then
    echo "ok: ${link} -> ${current} (已存在)"
    exit 0
  fi
  echo "error: ${link} 已是指向 ${current} 的符号链接,与预期的 ${target_rel} 不符;请人工确认后删除重建" >&2
  exit 1
fi

if [[ -e "${link}" ]]; then
  echo "error: ${link} 已存在且不是符号链接,拒绝覆盖" >&2
  exit 1
fi

ln -s "${target_rel}" "${link}"
echo "created: ${link} -> ${target_rel}"
