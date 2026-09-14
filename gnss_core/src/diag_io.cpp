#include "gnss_core/diag_io.hpp"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>

namespace gnss_core {

namespace {
std::string latlon_fields(const std::optional<LatLon>& pos) {
  if (!pos) return "lat=- lon=-";
  char buf[96];
  std::snprintf(buf, sizeof(buf), "lat=%.9f lon=%.9f", pos->lat, pos->lon);
  return buf;
}

std::string single_line(std::string s) {
  for (char& c : s) {
    if (c == '\n' || c == '\r') c = ' ';
  }
  return s;
}
}  // namespace

std::string format_utc_timestamp(double unix_s) {
  const long long total_ms = std::llround(unix_s * 1000.0);
  long long secs = total_ms / 1000;
  long long ms = total_ms % 1000;
  if (ms < 0) {
    ms += 1000;
    secs -= 1;
  }
  const std::time_t tt = static_cast<std::time_t>(secs);
  std::tm tm{};
  gmtime_r(&tt, &tm);
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d.%03lld", tm.tm_year + 1900,
                tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ms);
  return buf;
}

std::string events_log_header() {
  return "% gnss_core events.log (time=UTC)\n"
         "% OPEN : <time> OPEN <level> <code> lat=<deg|-> lon=<deg|-> <message>\n"
         "% CLOSE: <time> CLOSE <level> <code> lat=<deg|-> lon=<deg|-> opened=<time> duration_s=<s> "
         "reason=<recovered|shutdown> peak=<k=v;...|-> <message>\n";
}

std::string format_event_line(const EventTransition& e) {
  std::string line = format_utc_timestamp(e.t);
  line += e.kind == EventKind::Open ? " OPEN " : " CLOSE ";
  line += level_name(e.level);
  line += " " + e.code + " " + latlon_fields(e.pos);
  if (e.kind == EventKind::Close) {
    char dur[48];
    std::snprintf(dur, sizeof(dur), "%.1f", e.t - e.t_open);
    line += " opened=" + format_utc_timestamp(e.t_open) + " duration_s=" + dur +
            " reason=" + close_reason_name(e.reason) + " peak=";
    if (e.peak.empty()) {
      line += "-";
    } else {
      bool first = true;
      for (const auto& [key, value] : e.peak) {   // std::map:键字典序
        char kv[128];
        std::snprintf(kv, sizeof(kv), "%s%s=%.3f", first ? "" : ";", key.c_str(), value);
        line += kv;
        first = false;
      }
    }
  }
  line += " " + single_line(e.message);
  return line;
}

std::string base_pos_header() {
  return "% program : gnss_core base history\n"
         "% time=UTC\n"
         "%  UTC                    x-ecef(m)        y-ecef(m)        z-ecef(m)\n";
}

std::string format_base_history_line(double t, const Ecef& p) {
  char buf[128];
  std::snprintf(buf, sizeof(buf), "  %.4f %.4f %.4f", p.x, p.y, p.z);
  return format_utc_timestamp(t) + buf;
}

bool LineAppender::open(const std::string& path, const std::string& header) {
  close();
  std::error_code ec;
  const std::filesystem::path fp(path);
  if (fp.has_parent_path()) std::filesystem::create_directories(fp.parent_path(), ec);

  bool is_new = true;
  bool needs_newline = false;
  const auto size = std::filesystem::file_size(fp, ec);
  if (!ec && size > 0) {
    is_new = false;
    std::ifstream in(path, std::ios::binary);
    in.seekg(static_cast<std::streamoff>(size) - 1);
    char last = 0;
    if (!in.get(last)) return false;   // 读不到最后一个字节:宁可不写,也不往可能损坏的文件里追加
    needs_newline = last != '\n';
  }

  out_.open(path, std::ios::app | std::ios::binary);
  if (!out_.is_open()) return false;
  path_ = path;
  if (is_new) out_ << header;
  if (needs_newline) out_ << '\n';   // 上次掉电留下半行:补换行,不截断
  out_.flush();
  if (!out_) {
    close();
    return false;
  }
  return true;
}

bool LineAppender::append(const std::string& line) {
  if (!out_.is_open()) return false;
  if (line.find('\n') != std::string::npos || line.find('\r') != std::string::npos) return false;
  out_ << line << '\n';
  out_.flush();
  return static_cast<bool>(out_);
}

void LineAppender::close() {
  if (out_.is_open()) out_.close();
  out_.clear();
  path_.clear();
}

}  // namespace gnss_core
