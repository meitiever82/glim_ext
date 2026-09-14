#!/usr/bin/env bash
# record_gnss.sh —— 用 rosbag2 录制 GNSS 数据面的四路原始/半原始数据
# (spec §5.2:四路数据全部在总线上后,rosbag2 直接承担原始流记录与回放,
# 不需要额外的自定义落盘逻辑;.pos 摘要是给人/下游算法看的 1 Hz 精简版,
# 两者用途不同,不是互相替代关系,见 README「rosbag2 录制」一节)。
#
# 用法:
#   source install/setup.bash
#   bash src/glim_ext/gnss_bringup/scripts/record_gnss.sh
#
# 可用环境变量覆盖(默认值见下面每一行右侧注释):
#   GNSS_BAG_ROOT         录制输出的根目录,脚本会在其下建一个带时间戳的
#                         子目录作为这次录制的 -o 输出路径。
#                         默认: "$HOME/gnss_bags"
#   GNSS_BAG_TOPICS       要录制的话题清单,空格分隔的一个字符串(话题名本身
#                         不含空格,可以安全地按空格切分)。
#                         默认录制六个话题:两路裸流(rtcm_bridge 转发,类型都是
#                         gnss_msgs/RawStream,靠话题名区分,见 README)、
#                         三路 RtkFix(/gnss_cgi610/rtk_fix;/gnss_cgi610/rtk_fix_gpchc
#                         目前仓库里没有任何发布者,留在清单里是为了现场协议确认后
#                         不必改脚本;/rtkrcv_node/rtk_fix)、以及 rtkrcv_node 转发的
#                         $SAT 状态行。
#                         默认:
#                           /gnss/rtcm_corrections /gnss/raw_obs
#                           /gnss_cgi610/rtk_fix /gnss_cgi610/rtk_fix_gpchc
#                           /rtkrcv_node/rtk_fix /rtkrcv_node/stat
#   GNSS_BAG_MAX_DURATION_S  单个 bag 分卷的最长时长(秒),对应
#                         `ros2 bag record --max-bag-duration`。默认 86400
#                         (一天)——注意这是"从录制进程启动那一刻起满 N 秒就切卷",
#                         不是像 pos_writer 那样按 UTC 自然日对齐;长期跑着不重启
#                         的录制进程,分卷边界会逐渐偏离自然日,这与 README 里
#                         `.pos` 按 UTC 零点换文件是两套不同的机制,不要混淆。
#   GNSS_BAG_STORAGE      rosbag2 存储后端,对应 `--storage`。默认 sqlite3。
#
# 缺口(spec A6,轮 3 做,这里只记录,不实现):rosbag2 本身没有按保留天数
# 或磁盘水位自动清理旧 bag 的能力——这个脚本只管"怎么录",不管"录多了怎么
# 删"。长期运行必须由运维自行监控 GNSS_BAG_ROOT 所在磁盘的占用并手动/用外部
# 定时任务清理,否则会把磁盘写满。详见 README「rosbag2 录制」一节。
set -euo pipefail

GNSS_BAG_ROOT="${GNSS_BAG_ROOT:-$HOME/gnss_bags}"
GNSS_BAG_TOPICS="${GNSS_BAG_TOPICS:-/gnss/rtcm_corrections /gnss/raw_obs /gnss_cgi610/rtk_fix /gnss_cgi610/rtk_fix_gpchc /rtkrcv_node/rtk_fix /rtkrcv_node/stat}"
GNSS_BAG_MAX_DURATION_S="${GNSS_BAG_MAX_DURATION_S:-86400}"
GNSS_BAG_STORAGE="${GNSS_BAG_STORAGE:-sqlite3}"

# 话题名按空格切分成数组;GNSS_BAG_TOPICS 里的话题名本身不含空格,这里的
# word splitting 是有意为之(不是遗漏引号)。
# shellcheck disable=SC2206
topics=(${GNSS_BAG_TOPICS})

if [[ ${#topics[@]} -eq 0 ]]; then
  echo "record_gnss.sh: GNSS_BAG_TOPICS 为空,没有任何话题可录" >&2
  exit 1
fi

mkdir -p "${GNSS_BAG_ROOT}"
output_dir="${GNSS_BAG_ROOT}/gnss_$(date -u +%Y%m%d_%H%M%S)"

# 启动前逐一检查话题是否已经在总线上——不存在只警告、不阻止启动:
# 录制一个当前还没有发布者的话题是合法的(ros2 bag record 本身支持动态发现,
# 后续发布者一上线就会被录进去),链路(rtcm_bridge/rtkrcv_node/gnss_cgi610)
# 完全可能按不同顺序、先后起来。
if command -v ros2 >/dev/null 2>&1; then
  existing_topics="$(ros2 topic list 2>/dev/null || true)"
  for t in "${topics[@]}"; do
    if ! grep -qx -- "${t}" <<<"${existing_topics}"; then
      echo "record_gnss.sh: 警告——话题 ${t} 当前不在总线上,仍会加入录制清单" \
        "(链路可能稍后才起来;如果它长期不出现,检查对应节点是否已启动/话题名是否拼对)" >&2
    fi
  done
else
  echo "record_gnss.sh: 警告——找不到 ros2 命令,跳过话题存在性检查(是否已 source setup.bash?)" >&2
fi

echo "record_gnss.sh: 录制到 ${output_dir}(max-bag-duration=${GNSS_BAG_MAX_DURATION_S}s,storage=${GNSS_BAG_STORAGE})"
echo "record_gnss.sh: 话题清单: ${topics[*]}"

exec ros2 bag record \
  -o "${output_dir}" \
  --max-bag-duration "${GNSS_BAG_MAX_DURATION_S}" \
  --storage "${GNSS_BAG_STORAGE}" \
  "${topics[@]}"
