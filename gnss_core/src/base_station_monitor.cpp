#include "gnss_core/base_station_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace gnss_core {

namespace {
double median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  return n % 2 == 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

double distance(const Ecef& a, const Ecef& b) {
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}
}  // namespace

BaseStationMonitor::BaseStationMonitor(double warmup_s, std::optional<Ecef> baseline,
                                       std::optional<Ecef> last_history)
    : warmup_s_(warmup_s), baseline_(baseline), last_history_(last_history) {}

BaseFeedResult BaseStationMonitor::feed(double t, const Ecef& p) {
  BaseFeedResult r;
  if (!last_history_ || std::abs(p.x - last_history_->x) > 1e-3 ||
      std::abs(p.y - last_history_->y) > 1e-3 || std::abs(p.z - last_history_->z) > 1e-3) {
    r.history_changed = true;
    last_history_ = p;
  }
  if (!baseline_) {
    samples_.emplace_back(t, p);
    if (t - samples_.front().first < warmup_s_) return r;
    std::vector<double> xs, ys, zs;
    for (const auto& [ts, s] : samples_) {
      xs.push_back(s.x);
      ys.push_back(s.y);
      zs.push_back(s.z);
    }
    baseline_ = Ecef{median(xs), median(ys), median(zs)};
    samples_.clear();
    r.baseline_learned = true;
  }
  r.offset_m = distance(p, *baseline_);
  return r;
}

BaseFeedResult BaseStationMonitor::reset(const Ecef& p) {
  baseline_ = p;
  last_history_ = p;
  samples_.clear();
  BaseFeedResult r;
  r.offset_m = 0.0;
  r.baseline_learned = true;
  r.history_changed = true;
  return r;
}

std::optional<Ecef> read_base_baseline(const std::string& path) {
  std::ifstream in(path);
  if (!in) return std::nullopt;
  std::string line;
  if (!std::getline(in, line)) return std::nullopt;
  std::istringstream ss(line);
  Ecef p;
  char c1 = 0, c2 = 0;
  if (!(ss >> p.x >> c1 >> p.y >> c2 >> p.z) || c1 != ',' || c2 != ',') return std::nullopt;
  std::string rest;
  if (ss >> rest) return std::nullopt;   // 多余字段(比如第四个值)视为损坏
  return p;
}

bool write_base_baseline(const std::string& path, const Ecef& p) {
  std::error_code ec;
  const std::filesystem::path fp(path);
  if (fp.has_parent_path()) std::filesystem::create_directories(fp.parent_path(), ec);
  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) return false;
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%.4f,%.4f,%.4f\n", p.x, p.y, p.z);
    out << buf;
    out.flush();
    if (!out) return false;
  }
  std::filesystem::rename(tmp, fp, ec);   // 原子替换:崩溃时要么旧基线要么新基线
  if (ec) {
    std::filesystem::remove(tmp, ec);
    return false;
  }
  return true;
}

}  // namespace gnss_core
