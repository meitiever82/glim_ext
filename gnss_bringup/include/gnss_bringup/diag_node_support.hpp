#pragma once
// gnss_diag_node 的纯逻辑(轮 3b):ROS 消息 → gnss_core 类型、DiagnosticStatus 组装、按天追加写、
// 跨日期目录读 base.pos 末行、时钟回跳检测、控制点参数、配对失败提醒。节点只负责接线。
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <gnss_msgs/msg/rtk_fix.hpp>

#include "gnss_bringup/pos_rotation.hpp"
#include "gnss_core/base_station_monitor.hpp"
#include "gnss_core/diag_io.hpp"
#include "gnss_core/diagnosis_engine.hpp"
#include "gnss_core/retention.hpp"

namespace gnss_bringup {

inline constexpr const char* kDiagStatusName = "gnss_diag";

// RtkFix → 诊断用的解样本。0 表示源不提供的字段(ratio、gnss_time,见 RtkFix.msg)映射为空,
// 不能当成真实的 0 去判 ambiguity 或按历元配对。
inline gnss_core::SolutionSample to_solution_sample(const gnss_msgs::msg::RtkFix& m) {
  gnss_core::SolutionSample s;
  s.quality = m.quality <= static_cast<uint8_t>(gnss_core::Quality::FIXED)
                  ? static_cast<gnss_core::Quality>(m.quality)
                  : gnss_core::Quality::NONE;
  s.lat = m.latitude;
  s.lon = m.longitude;
  s.ns = m.sats_used;
  s.sdn = m.sigma_enu[1];   // sigma_enu 是 E/N/U
  s.sde = m.sigma_enu[0];
  s.age = m.diff_age;
  if (std::isfinite(m.ratio) && m.ratio > 0.0f) s.ratio = m.ratio;
  if (std::isfinite(m.gnss_time) && m.gnss_time > 0.0) s.epoch_t = m.gnss_time;
  return s;
}

inline uint8_t to_diagnostic_level(gnss_core::Level level) {
  using S = diagnostic_msgs::msg::DiagnosticStatus;
  switch (level) {
    case gnss_core::Level::Ok:
    case gnss_core::Level::Info: return S::OK;
    case gnss_core::Level::Warning: return S::WARN;
    case gnss_core::Level::Serious:
    case gnss_core::Level::Critical: return S::ERROR;
  }
  return S::ERROR;
}

namespace detail {
inline void add_value(diagnostic_msgs::msg::DiagnosticStatus& st, std::string key, std::string value) {
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = std::move(key);
  kv.value = std::move(value);
  st.values.push_back(std::move(kv));
}

inline std::string fixed3(double v) {
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%.3f", v);
  return buf;
}
}  // namespace detail

// 一拍的诊断状态:level 取全部结论里最严重的一条,message/status_code 取优先级最高的一条
// (DiagnosisResult::status());values 列出已开事件、偏差、阈值与每条结论。
inline diagnostic_msgs::msg::DiagnosticStatus make_diagnostic_status(const gnss_core::TickResult& r,
                                                                     const std::vector<std::string>& open_codes) {
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = kDiagStatusName;
  st.hardware_id = "gnss";
  if (r.result.verdicts.empty()) {   // evaluate_rules 保证非空;这里不依赖它(status() 对空结果是未定义行为)
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message = "规则链没有输出";
    return st;
  }
  gnss_core::Level worst = gnss_core::Level::Ok;
  for (const auto& v : r.result.verdicts) worst = std::max(worst, v.level);
  st.level = to_diagnostic_level(worst);
  const auto& head = r.result.status();
  st.message = head.message;
  detail::add_value(st, "status_code", head.code);
  std::string open;
  for (const auto& c : open_codes) open += (open.empty() ? "" : ",") + c;
  detail::add_value(st, "open_events", open.empty() ? "-" : open);
  detail::add_value(st, "divergence_m",
                    r.divergence.divergence_m ? detail::fixed3(*r.divergence.divergence_m) : "-");
  detail::add_value(st, "divergence_threshold_m", detail::fixed3(r.divergence.threshold_m));
  for (const auto& v : r.result.verdicts) {
    detail::add_value(st, "verdict." + v.code, std::string(gnss_core::level_name(v.level)) + " " + v.message);
  }
  return st;
}

inline diagnostic_msgs::msg::DiagnosticStatus make_startup_grace_status(double remaining_s) {
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = kDiagStatusName;
  st.hardware_id = "gnss";
  st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  char buf[96];
  std::snprintf(buf, sizeof(buf), "启动宽限期(剩余 %.0f s),暂不判定", std::max(0.0, remaining_s));
  st.message = buf;
  return st;
}

// <root>/YYYYMMDD/<filename> 的逐行追加写,跨 UTC 零点自动换文件,新文件先写 header。
// 打开或写入失败时本行丢弃并返回 false(调用方节流报错),并关闭文件,下一行重新尝试打开。
class DayFileAppender {
public:
  DayFileAppender(std::string root, std::string filename, std::string header)
      : root_(std::move(root)), filename_(std::move(filename)), header_(std::move(header)) {}

  bool append(double t, const std::string& line) {
    const std::string path = day_file_path(root_, filename_, t);
    if (path.empty()) return false;
    if (!out_.is_open() || out_.path() != path) {
      if (!out_.open(path, header_)) return false;
    }
    if (!out_.append(line)) {
      out_.close();
      return false;
    }
    return true;
  }
  void close() { out_.close(); }
  const std::string& current_path() const { return out_.path(); }

private:
  std::string root_, filename_, header_;
  gnss_core::LineAppender out_;
};

// base.pos 数据行:"YYYY/MM/DD HH:MM:SS.sss x y z"(gnss_core::format_base_history_line)。
// 注释行、空行、字段不全(掉电留下的半行)、非有限坐标都返回空。
inline std::optional<gnss_core::Ecef> parse_base_history_line(const std::string& line) {
  if (line.empty() || line[0] == '%') return std::nullopt;
  std::istringstream in(line);
  std::string date, time;
  gnss_core::Ecef p;
  if (!(in >> date >> time >> p.x >> p.y >> p.z)) return std::nullopt;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return std::nullopt;
  return p;
}

// 设计决定 6:从 <root> 下日期最新、含有效数据行的 YYYYMMDD/base.pos 取最后一个有效行;
// 找不到(首次运行、root 不存在)返回空——引擎因此会多写一行历史,无害。
inline std::optional<gnss_core::Ecef> read_last_base_history(const std::string& root) {
  namespace fs = std::filesystem;
  std::vector<std::pair<int, std::string>> days;
  std::error_code ec;
  for (fs::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec)) {
    const std::string name = it->path().filename().string();
    if (const auto d = gnss_core::parse_day_dir_date(name)) days.emplace_back(*d, name);
  }
  std::sort(days.rbegin(), days.rend());
  for (const auto& [date, name] : days) {
    std::ifstream in(fs::path(root) / name / "base.pos");
    std::optional<gnss_core::Ecef> last;
    for (std::string line; std::getline(in, line);) {
      if (const auto p = parse_base_history_line(line)) last = p;
    }
    if (last) return last;
  }
  return std::nullopt;
}

// 引擎要求时间单调不减(3a 遗留 B),ROS 时间(尤其 use_sim_time 回放)可能回退:
//   - 回退不超过 tolerance_s:夹到上一次的值,不算事件;
//   - 回退超过 tolerance_s:jumped=true,本次时间成为新起点,调用方关闭事件并重建引擎(设计决定 2)。
// 向前跳不处理(引擎按新鲜度自然过期)。
struct ClockStep {
  double t = 0.0;
  bool jumped = false;
};

class MonotonicClockGuard {
public:
  explicit MonotonicClockGuard(double tolerance_s) : tolerance_s_(tolerance_s) {}
  ClockStep step(double now) {
    if (!last_) {
      last_ = now;
      return {now, false};
    }
    if (now < *last_ - tolerance_s_) {
      last_ = now;
      return {now, true};
    }
    last_ = std::max(*last_, now);
    return {*last_, false};
  }
  std::optional<double> last() const { return last_; }

private:
  double tolerance_s_;
  std::optional<double> last_;
};

// 设计决定 7:控制点参数是三个等长数组(ROS 参数不支持结构体数组)
inline std::vector<gnss_core::ControlPoint> control_points_from_params(const std::vector<std::string>& names,
                                                                       const std::vector<double>& lat,
                                                                       const std::vector<double>& lon) {
  if (names.size() != lat.size() || names.size() != lon.size()) {
    throw std::invalid_argument("control_points.names/.lat/.lon 长度必须相同(收到 " +
                                std::to_string(names.size()) + "/" + std::to_string(lat.size()) + "/" +
                                std::to_string(lon.size()) + ")");
  }
  std::vector<gnss_core::ControlPoint> out;
  for (size_t i = 0; i < names.size(); ++i) {
    const std::string idx = "[" + std::to_string(i) + "]";
    if (names[i].empty()) throw std::invalid_argument("control_points.names" + idx + " 不能为空");
    if (!std::isfinite(lat[i]) || std::abs(lat[i]) > 90.0) {
      throw std::invalid_argument("control_points.lat" + idx + " 必须在 [-90, 90]");
    }
    if (!std::isfinite(lon[i]) || std::abs(lon[i]) > 180.0) {
      throw std::invalid_argument("control_points.lon" + idx + " 必须在 [-180, 180]");
    }
    gnss_core::ControlPoint cp;
    cp.name = names[i];
    cp.lat = lat[i];
    cp.lon = lon[i];
    out.push_back(std::move(cp));
  }
  return out;
}

// 3a 遗留 B:两路解都在按时到达、却连续 streak_ticks 拍没配上对(历元始终相差超过
// divergence_epoch_max_dt_s,或一路缺历元时刻而到达时刻相差太大)时,device_divergence 会
// 静默失效。update 在连续失败达到门限的那一拍返回 true(每段只一次);配上或有一路没到时复位。
class UnpairedWatch {
public:
  explicit UnpairedWatch(int streak_ticks) : limit_(streak_ticks) {}
  bool update(bool both_streams_live, bool paired) {
    if (!both_streams_live || paired) {
      streak_ = 0;
      warned_ = false;
      return false;
    }
    if (++streak_ >= limit_ && !warned_) {
      warned_ = true;
      return true;
    }
    return false;
  }

private:
  int limit_;
  int streak_ = 0;
  bool warned_ = false;
};

}  // namespace gnss_bringup
