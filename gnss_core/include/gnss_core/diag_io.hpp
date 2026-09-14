#pragma once
// 诊断落盘格式(spec §5.3 目录布局、D2 events.log、D3 base.pos)与逐行追加写:
// 进程崩溃安全（每行 flush；不 fsync，掉电可能丢最后几行或留下尾部垃圾）。
#include <fstream>
#include <string>

#include "gnss_core/base_station_monitor.hpp"
#include "gnss_core/event_book.hpp"

namespace gnss_core {

std::string format_utc_timestamp(double unix_s);

std::string events_log_header();
std::string format_event_line(const EventTransition& e);

std::string base_pos_header();
std::string format_base_history_line(double t, const Ecef& p);

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
