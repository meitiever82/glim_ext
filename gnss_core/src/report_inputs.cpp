#include "gnss_core/report_inputs.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "gnss_core/diag_io.hpp"
#include "gnss_core/retention.hpp"

namespace gnss_core {

namespace {
namespace fs = std::filesystem;

bool in_window(double t, const ReportWindow& w) { return t >= w.t0 && t < w.t1; }

// root 下日期落在 [first, last] 的 YYYYMMDD 目录,按日期升序
std::vector<std::pair<int, fs::path>> day_dirs(const std::string& root, int first, int last) {
  std::error_code ec;
  if (!fs::is_directory(root, ec)) throw std::runtime_error("报告根目录不存在或不是目录: " + root);
  std::vector<std::pair<int, fs::path>> out;
  fs::directory_iterator it(root, ec);
  if (ec) throw std::runtime_error("无法遍历 " + root + ": " + ec.message());
  for (; !ec && it != fs::directory_iterator(); it.increment(ec)) {
    std::error_code type_ec;
    if (!it->is_directory(type_ec) || type_ec) continue;
    const auto date = parse_day_dir_date(it->path().filename().string());
    if (date && *date >= first && *date <= last) out.emplace_back(*date, it->path());
  }
  std::sort(out.begin(), out.end());
  return out;
}

bool read_lines(const fs::path& path, std::vector<std::string>& lines) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  for (std::string line; std::getline(in, line);) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    lines.push_back(std::move(line));
  }
  return !in.bad();
}

bool is_content_line(const std::string& line) { return !line.empty() && line[0] != '%'; }

// t0 之前时刻最大的一条有效行并入 best(没有则不动)
void keep_latest_before(double t, const Ecef& p, double t0, std::optional<BaseSample>& best) {
  if (t < t0 && (!best || t > best->t)) best = BaseSample{t, p};
}

// 设计决定 9:按文件顺序配对 OPEN/CLOSE
std::vector<ReportEvent> pair_events(const std::vector<EventLogLine>& lines) {
  std::map<std::string, ReportEvent> pending;
  std::vector<ReportEvent> all;
  for (const auto& l : lines) {
    if (l.kind == EventKind::Open) {
      auto it = pending.find(l.code);
      if (it != pending.end()) {   // 同一规则码又开了:上一条没等到关闭行(进程中断后重启)
        all.push_back(it->second);
        pending.erase(it);
      }
      ReportEvent e;
      e.code = l.code;
      e.level = l.level;
      e.message = l.message;
      e.t_open = l.t;
      e.pos = l.pos;
      pending.emplace(l.code, std::move(e));
      continue;
    }
    ReportEvent e;
    auto it = pending.find(l.code);
    if (it != pending.end() && std::abs(it->second.t_open - l.t_open) < 0.002) {
      e = std::move(it->second);
      pending.erase(it);
      if (!e.pos) e.pos = l.pos;
    } else {
      e.code = l.code;
      e.level = l.level;
      e.message = l.message;
      e.t_open = l.t_open;
      e.pos = l.pos;
    }
    e.t_close = l.t;
    e.close_reason = l.reason;
    e.peak = l.peak;
    all.push_back(std::move(e));
  }
  for (auto& [code, e] : pending) all.push_back(std::move(e));
  return all;
}
}  // namespace

ReportInputs load_report_inputs(const std::string& root, const ReportWindow& window,
                                const PosReadOptions& pos_options) {
  ReportInputs in;
  const int pos_first = utc_yyyymmdd(window.t0);
  const int first = utc_yyyymmdd(window.t0 - 86400.0);
  const int last = utc_yyyymmdd(window.t1 - 1e-3);
  std::vector<EventLogLine> event_lines;

  for (const auto& [date, dir] : day_dirs(root, first, last)) {
    if (date >= pos_first) {
      std::vector<fs::path> pos_files;
      std::error_code ec;
      for (fs::directory_iterator it(dir, ec); !ec && it != fs::directory_iterator(); it.increment(ec)) {
        std::error_code type_ec;
        if (!it->is_regular_file(type_ec) || type_ec) continue;
        const fs::path& p = it->path();
        if (p.extension() == ".pos" && p.filename() != "base.pos" && !p.stem().empty()) pos_files.push_back(p);
      }
      std::sort(pos_files.begin(), pos_files.end());
      for (const auto& p : pos_files) {
        try {
          auto records = read_pos(p.string(), pos_options);
          auto& dst = in.sources[p.stem().string()];
          for (auto& r : records) {
            if (in_window(r.stamp, window)) dst.push_back(std::move(r));
          }
        } catch (const std::exception& e) {
          in.warnings.push_back("读取 " + p.string() + " 失败: " + e.what());
        }
      }
    }

    const fs::path events_path = dir / "events.log";
    std::error_code ec;
    if (fs::exists(events_path, ec)) {
      std::vector<std::string> lines;
      if (!read_lines(events_path, lines)) {
        in.warnings.push_back("读取 " + events_path.string() + " 失败");
      } else {
        int bad = 0;
        for (const auto& line : lines) {
          if (auto parsed = parse_event_line(line)) {
            event_lines.push_back(std::move(*parsed));
          } else if (is_content_line(line)) {
            ++bad;
          }
        }
        if (bad > 0) {
          in.warnings.push_back(events_path.string() + " 中 " + std::to_string(bad) + " 行无法解析(可能是掉电留下的半行)");
        }
      }
    }

    const fs::path base_path = dir / "base.pos";
    if (fs::exists(base_path, ec)) {
      std::vector<std::string> lines;
      if (!read_lines(base_path, lines)) {
        in.warnings.push_back("读取 " + base_path.string() + " 失败");
      } else {
        int bad = 0;
        for (const auto& line : lines) {
          if (const auto parsed = parse_base_history_line(line)) {
            if (in_window(parsed->first, window)) in.base_history.push_back(BaseSample{parsed->first, parsed->second});
            keep_latest_before(parsed->first, parsed->second, window.t0, in.base_before_window);
          } else if (is_content_line(line)) {
            ++bad;
          }
        }
        if (bad > 0) {
          in.warnings.push_back(base_path.string() + " 中 " + std::to_string(bad) + " 行无法解析(可能是掉电留下的半行)");
        }
      }
    }
  }

  // 基站坐标稳定性的基准:窗口前最后一条有效行。上面已扫过 [t0 前一天, t1 所在日];
  // 还没有就像 gnss_bringup 的 read_last_base_history 那样往更早的日期目录倒着找,
  // 找到第一个有有效行的目录就停(base.pos 只在坐标变化时才写,几天没变很正常)。
  // 这里读不了的文件不重复报警告(与节点启动时的行为一致:当作没有记录)。
  if (!in.base_before_window) {
    auto older = day_dirs(root, 0, first - 1);
    for (auto it = older.rbegin(); it != older.rend() && !in.base_before_window; ++it) {
      std::vector<std::string> lines;
      std::error_code ec;
      const fs::path base_path = it->second / "base.pos";
      if (!fs::exists(base_path, ec) || !read_lines(base_path, lines)) continue;
      for (const auto& line : lines) {
        if (const auto parsed = parse_base_history_line(line)) {
          keep_latest_before(parsed->first, parsed->second, window.t0, in.base_before_window);
        }
      }
    }
  }

  for (auto it = in.sources.begin(); it != in.sources.end();) {
    if (it->second.empty()) {
      it = in.sources.erase(it);
    } else {
      std::stable_sort(it->second.begin(), it->second.end(),
                       [](const PosRecord& a, const PosRecord& b) { return a.stamp < b.stamp; });
      ++it;
    }
  }
  std::stable_sort(in.base_history.begin(), in.base_history.end(),
                   [](const BaseSample& a, const BaseSample& b) { return a.t < b.t; });

  for (auto& e : pair_events(event_lines)) {
    if (e.t_open < window.t1 && (!e.t_close || *e.t_close >= window.t0)) in.events.push_back(std::move(e));
  }
  std::stable_sort(in.events.begin(), in.events.end(), [](const ReportEvent& a, const ReportEvent& b) {
    return std::tie(a.t_open, a.code) < std::tie(b.t_open, b.code);
  });
  return in;
}

}  // namespace gnss_core
