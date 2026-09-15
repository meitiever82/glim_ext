#pragma once
// gnss_cleanup_node 的参数校验与一轮清理(轮 3b 设计决定 8)。不碰 ROS:节点只负责定时与打日志。
//   - 两个根目录:bag_root(rosbag2 录包,gnss_YYYYMMDD_HHMMSS)与 pos_root(.pos 与诊断文件,YYYYMMDD);
//     各自可为空(不清),不能都空。
//   - 先扫录包根目录再扫 pos 根目录:两者通常在同一块盘上共用水位,录包占盘大,先删它。
//   - 根目录不存在(比如还没录过包)视为跳过,不算错误;存在但遍历失败(比如是个普通文件)才是错误。
//   - 每个根目录清完后复查所在盘的用量,仍高于水位由调用方报警(最新一项与今天的数据永不删)。
#include <cmath>
#include <filesystem>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "gnss_core/retention.hpp"

namespace gnss_bringup {

struct CleanupParams {
  std::string bag_root;
  std::string pos_root;
  int retention_days = 14;
  double watermark_pct = 85.0;
  double interval_s = 3600.0;
};

inline void validate_cleanup_params(const CleanupParams& p) {
  if (p.bag_root.empty() && p.pos_root.empty()) {
    throw std::invalid_argument("bag_root 与 pos_root 不能都为空");
  }
  if (p.retention_days < 1) throw std::invalid_argument("retention_days 必须 >= 1");
  if (!std::isfinite(p.watermark_pct) || p.watermark_pct <= 0.0 || p.watermark_pct > 100.0) {
    throw std::invalid_argument("watermark_pct 必须在 (0, 100]");
  }
  if (!std::isfinite(p.interval_s) || p.interval_s < 1.0) throw std::invalid_argument("interval_s 必须 >= 1");
}

struct RootCleanupResult {
  std::string kind;     // "bag" 或 "pos"
  std::string root;
  bool missing = false; // 根目录不存在,本轮跳过
  gnss_core::CleanupReport report;
  std::optional<double> used_pct_after;   // 清完后所在盘的用量;查不到为空
};

inline std::vector<RootCleanupResult> run_cleanup_pass(
    const CleanupParams& p, int today_yyyymmdd,
    const std::function<std::optional<double>(const std::string&)>& used_pct = gnss_core::disk_used_pct) {
  std::vector<RootCleanupResult> out;
  const auto one = [&](const char* kind, const std::string& root,
                       const std::function<std::optional<int>(const std::string&)>& parse_date) {
    if (root.empty()) return;
    RootCleanupResult r;
    r.kind = kind;
    r.root = root;
    std::error_code ec;
    if (!std::filesystem::exists(root, ec) && !ec) {
      r.missing = true;
    } else {
      r.report = gnss_core::cleanup_dated_root(root, parse_date, today_yyyymmdd, p.retention_days, p.watermark_pct);
      r.used_pct_after = used_pct(root);
    }
    out.push_back(std::move(r));
  };
  one("bag", p.bag_root, gnss_core::parse_bag_dir_date);
  one("pos", p.pos_root, gnss_core::parse_day_dir_date);
  return out;
}

inline bool over_watermark(const RootCleanupResult& r, double watermark_pct) {
  return r.used_pct_after && *r.used_pct_after > watermark_pct;
}

}  // namespace gnss_bringup
