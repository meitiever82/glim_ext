#include "gnss_core/rtk_fix_buffer.hpp"
#include <algorithm>

namespace gnss_core {

void RtkFixBuffer::push(const RtkFixSample& s) {
  if (buf_.empty() || s.stamp >= buf_.back().stamp) { buf_.push_back(s); return; }
  auto it = std::lower_bound(buf_.begin(), buf_.end(), s.stamp,
      [](const RtkFixSample& a, double t){ return a.stamp < t; });
  buf_.insert(it, s);
}

std::optional<RtkFixSample> RtkFixBuffer::interpolate(double t, double max_gap_s) const {
  if (buf_.size() < 2) return std::nullopt;
  if (t < buf_.front().stamp || t > buf_.back().stamp) return std::nullopt;
  auto right = std::lower_bound(buf_.begin(), buf_.end(), t,
      [](const RtkFixSample& a, double tt){ return a.stamp < tt; });
  if (right == buf_.begin()) return *right;          // t == front
  auto left = right - 1;
  const double tl = left->stamp, tr = right->stamp;
  if (tr - tl > max_gap_s) return std::nullopt;      // 两端间隔过大,拒绝跨断链插值
  const double p = (tr > tl) ? (t - tl) / (tr - tl) : 0.0;
  RtkFixSample out;
  out.stamp = t;
  out.header_stamp = (1 - p) * left->header_stamp + p * right->header_stamp;
  out.gnss_time = (1 - p) * left->gnss_time + p * right->gnss_time;
  out.lat = (1 - p) * left->lat + p * right->lat;
  out.lon = (1 - p) * left->lon + p * right->lon;
  out.alt = (1 - p) * left->alt + p * right->alt;
  out.sigma_enu = (1 - p) * left->sigma_enu + p * right->sigma_enu;
  out.diff_age = (1 - p) * left->diff_age + p * right->diff_age;
  out.sats_used = std::min(left->sats_used, right->sats_used);
  out.quality = static_cast<Quality>(std::min(
      static_cast<uint8_t>(left->quality), static_cast<uint8_t>(right->quality)));
  out.heading = (1 - p) * left->heading + p * right->heading;
  out.heading_valid = left->heading_valid && right->heading_valid;
  return out;
}

void RtkFixBuffer::prune(double horizon_s, double now) {
  // now 为数据时间(见头文件),不是壁钟
  const double cutoff = now - horizon_s;
  while (!buf_.empty() && buf_.front().stamp < cutoff) buf_.pop_front();
}

}  // namespace gnss_core
