#pragma once
// 报告工具的输入装载(spec §3 F2,轮 4a):从 <root>/YYYYMMDD/ 读取时间窗内的 .pos、events.log、base.pos。
// 只读文件、配对事件,不做统计(统计在 report_stats.hpp)。
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "gnss_core/base_station_monitor.hpp"
#include "gnss_core/event_book.hpp"
#include "gnss_core/pos_io.hpp"

namespace gnss_core {

// 报告时间窗:UTC unix 秒,半开区间 [t0, t1)
struct ReportWindow {
  double t0 = 0.0;
  double t1 = 0.0;
};

// 一次诊断事件:events.log 里 OPEN 行与 CLOSE 行配对后的结果
struct ReportEvent {
  std::string code;
  std::string level;                    // ok/info/warning/serious/critical
  std::string message;
  double t_open = 0.0;
  std::optional<double> t_close;        // 空:没找到关闭行(事件仍在进行,或进程在关闭前中断)
  std::string close_reason;             // recovered / shutdown;未关闭为空
  std::optional<LatLon> pos;            // 开启时位置;找不到 OPEN 行时取 CLOSE 行的位置
  std::map<std::string, double> peak;   // 仅已关闭事件
};

struct BaseSample {
  double t = 0.0;
  Ecef p;
};

struct ReportInputs {
  std::map<std::string, std::vector<PosRecord>> sources;   // 源名(.pos 文件名去扩展名) → 窗口内记录,时间升序
  std::vector<ReportEvent> events;                          // 与窗口有交集的事件,按开启时刻升序
  std::vector<BaseSample> base_history;                     // 窗口内基站坐标史,时间升序
  std::vector<std::string> warnings;                        // 读不了的文件、解析不了的行等,不中断装载
};

// root 不是目录时抛 std::runtime_error。
// .pos 读 [t0 所在日, t1 所在日] 的目录;events.log 与 base.pos 多读 t0 前一天,
// 以便找到窗口开始前开启、窗口内关闭的事件的 OPEN 行(设计决定 9、10)。
ReportInputs load_report_inputs(const std::string& root, const ReportWindow& window,
                                const PosReadOptions& pos_options = {});

}  // namespace gnss_core
