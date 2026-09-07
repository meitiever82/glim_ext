#pragma once
#include <deque>
#include <limits>
#include <optional>
#include "gnss_core/types.hpp"

namespace gnss_core {

// 按 stamp 递增维护的 RTK 样本缓冲,支持按时间线性插值。
// 所有时间都是"数据时间"(样本的 stamp),与壁钟无关。
class RtkFixBuffer {
public:
  void push(const RtkFixSample& s);              // 按 stamp 递增维护;乱序到达则插到正确位置

  // t 落在两样本之间 → 线性插值 lat/lon/alt/sigma/diff_age/heading/header_stamp/gnss_time,
  // quality 取两端较差者(数值较小者),sats_used 取较小者;
  // t 越界、缓冲不足两个样本、或左右样本间隔 > max_gap_s → std::nullopt。
  // max_gap_s 默认无穷(不限制);在线场景建议传 2~3 s,避免跨越长时间断链的假插值。
  std::optional<RtkFixSample> interpolate(
      double t, double max_gap_s = std::numeric_limits<double>::infinity()) const;

  // 丢弃 stamp < now - horizon_s 的样本。
  // 注意:now 是**数据时间**(通常传 latest_stamp()),不是壁钟。
  // 后台线程无壁钟意义,壳里不得传 ros::now / rclcpp::Clock::now。
  void prune(double horizon_s, double now);

  // 最新样本的 stamp(数据时间);空缓冲返回 0。供 prune 的 now 使用。
  double latest_stamp() const { return buf_.empty() ? 0.0 : buf_.back().stamp; }
  // 最旧样本的 stamp(数据时间);空缓冲返回 0。
  double oldest_stamp() const { return buf_.empty() ? 0.0 : buf_.front().stamp; }

  size_t size() const { return buf_.size(); }
private:
  std::deque<RtkFixSample> buf_;
};

}  // namespace gnss_core
