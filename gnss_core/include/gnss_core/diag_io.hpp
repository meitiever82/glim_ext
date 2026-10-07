#pragma once
// 诊断落盘格式(spec §5.3 目录布局、D2 events.log、D3 base.pos)与逐行追加写:
// 进程崩溃安全（每行 flush；不 fsync，掉电可能丢最后几行或留下尾部垃圾）；读回函数供轮 4a 报告工具使用。
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <utility>

#include "gnss_core/base_station_monitor.hpp"
#include "gnss_core/event_book.hpp"

namespace gnss_core {

std::string format_utc_timestamp(double unix_s);

std::string events_log_header();
std::string format_event_line(const EventTransition& e);

std::string base_pos_header();
std::string format_base_history_line(double t, const Ecef& p);

// ---- 读回(与上面的写出格式互逆) ----
// 两个解析函数都容忍行尾的 '\r' / '\n'(CRLF 文件),字段之间可以是任意空格/制表符。
// events.log 的一行。注释行(%)、空行、字段不全(掉电留下的半行)、类型/级别/关闭原因不认识时解析失败。
struct EventLogLine {
  EventKind kind = EventKind::Open;
  double t = 0.0;                        // 行时刻(UTC unix 秒)
  std::string level;                     // ok/info/warning/serious/critical
  std::string code;
  std::optional<LatLon> pos;
  std::string message;
  double t_open = 0.0;                   // OPEN 行等于 t;CLOSE 行取 opened=
  double duration_s = 0.0;               // 仅 CLOSE
  std::string reason;                    // 仅 CLOSE:recovered / shutdown
  std::map<std::string, double> peak;    // 仅 CLOSE
};
std::optional<EventLogLine> parse_event_line(const std::string& line);

// base.pos 的一行 → (UTC unix 秒, ECEF);注释行、半行、非有限坐标返回空。
// 时间 + x y z 之后再多出任何字段(哪怕是合法数字)也视为损坏、返回空——
// 格式由 format_base_history_line 独家写出,多出字段只可能是两行粘连或文件被改写;
// gnss_bringup 的 read_last_base_history 委托到这里,规则相同。
std::optional<std::pair<double, Ecef>> parse_base_history_line(const std::string& line);

class LineAppender {
public:
  bool open(const std::string& path, const std::string& header);
  bool append(const std::string& line);
  void close();
  bool is_open() const { return out_.is_open(); }
  const std::string& path() const { return path_; }

private:
  std::ofstream out_;
  std::string path_;
};

}  // namespace gnss_core
