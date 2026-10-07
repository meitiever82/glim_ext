#include "gnss_core/diag_io.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>

#include "gnss_core/pos_io.hpp"   // parse_utc_date_time

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

bool is_separator(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

// 取下一个以空白分隔的词,pos 前进到词尾。'\r' '\n' 也算分隔符:CRLF 文件 getline 后行尾留着 '\r',
// 调用方(比如 gnss_bringup 的 read_last_base_history)不一定先剥掉。
bool next_token(const std::string& s, size_t& pos, std::string& tok) {
  while (pos < s.size() && is_separator(s[pos])) ++pos;
  if (pos >= s.size()) return false;
  const size_t start = pos;
  while (pos < s.size() && !is_separator(s[pos])) ++pos;
  tok = s.substr(start, pos - start);
  return true;
}

bool strip_prefix(const std::string& tok, const char* prefix, std::string& rest) {
  const size_t n = std::strlen(prefix);
  if (tok.compare(0, n, prefix) != 0) return false;
  rest = tok.substr(n);
  return true;
}

// 整个字符串必须是一个有限浮点数
bool parse_finite(const std::string& s, double& out) {
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  out = std::strtod(s.c_str(), &end);
  return end == s.c_str() + s.size() && errno == 0 && std::isfinite(out);
}

bool known_level(const std::string& s) {
  return s == "ok" || s == "info" || s == "warning" || s == "serious" || s == "critical";
}

// "k=v;k=v" → map;"-" 表示空
bool parse_peak(const std::string& s, std::map<std::string, double>& out) {
  if (s == "-") return true;
  size_t start = 0;
  while (start <= s.size()) {
    const size_t end = std::min(s.find(';', start), s.size());
    const std::string kv = s.substr(start, end - start);
    const size_t eq = kv.find('=');
    double v = 0.0;
    if (eq == std::string::npos || eq == 0 || !parse_finite(kv.substr(eq + 1), v)) return false;
    out[kv.substr(0, eq)] = v;
    start = end + 1;
  }
  return true;
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

std::optional<EventLogLine> parse_event_line(const std::string& line) {
  if (line.empty() || line[0] == '%') return std::nullopt;
  EventLogLine e;
  size_t pos = 0;
  std::string date, time, kind, lat_tok, lon_tok, lat_s, lon_s;
  if (!next_token(line, pos, date) || !next_token(line, pos, time) || !parse_utc_date_time(date, time, e.t)) {
    return std::nullopt;
  }
  if (!next_token(line, pos, kind)) return std::nullopt;
  if (kind == "OPEN") {
    e.kind = EventKind::Open;
  } else if (kind == "CLOSE") {
    e.kind = EventKind::Close;
  } else {
    return std::nullopt;
  }
  if (!next_token(line, pos, e.level) || !known_level(e.level)) return std::nullopt;
  if (!next_token(line, pos, e.code)) return std::nullopt;
  if (!next_token(line, pos, lat_tok) || !next_token(line, pos, lon_tok) ||
      !strip_prefix(lat_tok, "lat=", lat_s) || !strip_prefix(lon_tok, "lon=", lon_s)) {
    return std::nullopt;
  }
  if (lat_s != "-" || lon_s != "-") {
    double lat = 0.0, lon = 0.0;
    if (!parse_finite(lat_s, lat) || !parse_finite(lon_s, lon)) return std::nullopt;
    e.pos = LatLon{lat, lon};
  }
  e.t_open = e.t;
  if (e.kind == EventKind::Close) {
    std::string tok, opened_date, opened_time, value;
    if (!next_token(line, pos, tok) || !strip_prefix(tok, "opened=", opened_date)) return std::nullopt;
    if (!next_token(line, pos, opened_time) || !parse_utc_date_time(opened_date, opened_time, e.t_open)) {
      return std::nullopt;
    }
    if (!next_token(line, pos, tok) || !strip_prefix(tok, "duration_s=", value) || !parse_finite(value, e.duration_s)) {
      return std::nullopt;
    }
    if (!next_token(line, pos, tok) || !strip_prefix(tok, "reason=", e.reason) ||
        (e.reason != "recovered" && e.reason != "shutdown")) {
      return std::nullopt;
    }
    if (!next_token(line, pos, tok) || !strip_prefix(tok, "peak=", value) || !parse_peak(value, e.peak)) {
      return std::nullopt;
    }
  }
  // format_event_line 用一个空格把结论接在最后;结论可以为空(行尾只剩这个空格或什么都没有)
  if (pos < line.size() && line[pos] == ' ') ++pos;
  e.message = line.substr(pos);
  while (!e.message.empty() && (e.message.back() == '\r' || e.message.back() == '\n')) e.message.pop_back();
  return e;
}

std::optional<std::pair<double, Ecef>> parse_base_history_line(const std::string& line) {
  if (line.empty() || line[0] == '%') return std::nullopt;
  size_t pos = 0;
  std::string date, time, xs, ys, zs, extra;
  double t = 0.0;
  Ecef p;
  if (!next_token(line, pos, date) || !next_token(line, pos, time) || !parse_utc_date_time(date, time, t)) {
    return std::nullopt;
  }
  if (!next_token(line, pos, xs) || !next_token(line, pos, ys) || !next_token(line, pos, zs) ||
      !parse_finite(xs, p.x) || !parse_finite(ys, p.y) || !parse_finite(zs, p.z)) {
    return std::nullopt;
  }
  if (next_token(line, pos, extra)) return std::nullopt;   // 多余字段视为损坏
  return std::make_pair(t, p);
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
