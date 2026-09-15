#pragma once
// 报告统计(spec §3 F2,轮 4a)。口径移植自 rtk-monitor report.py,
// 差异见 docs/gnss/plans/2026-09-16-round4a-report-tool.md「设计决定」5–11。纯函数,不读文件。
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "gnss_core/diagnosis.hpp"
#include "gnss_core/report_inputs.hpp"

namespace gnss_core {

struct ReportParams {
  ReportWindow window;
  std::vector<ControlPoint> control_points;
  double abs_ref_radius_m = 3.0;         // 经过控制点的判定半径
  double abs_ref_max_m = 0.2;            // 绝对基准告警阈值
  double base_shift_m = 0.1;             // 基站坐标变动告警阈值
  double pair_tol_s = 0.5;               // 610 与独立解按时间配对的容差(含边界)
  std::size_t max_track_points = 4000;   // 轨迹图每源最多点数
  double track_gap_s = 5.0;              // 相邻记录间隔超过此值时断开折线
};

// 按 RTKLIB Q 计数:1 fixed、2 float、4 dgps、5 single,其余计 other
struct QualityCounts {
  int fixed = 0, floating = 0, dgps = 0, single = 0, other = 0;
};

struct HourBucket {
  double t_start = 0.0;              // UTC 整点
  int epochs = 0;
  std::optional<double> fix_ratio;   // 该小时没有记录时为空
};

struct SourceStats {
  std::string name;
  int epochs = 0;
  std::optional<double> fix_ratio;
  QualityCounts counts;
  std::vector<HourBucket> hourly;    // 覆盖整个时间窗的每个 UTC 小时
};

struct DivergenceStats {
  std::string device;                // can / gpchc
  std::string reference = "rtkrcv";
  int n = 0;
  std::optional<double> max_m, mean_m;
};

struct AbsRefSample {
  double t = 0.0;
  std::string source;
  std::string control_point;
  double dev_m = 0.0;
};

struct AbsRefStats {
  std::vector<AbsRefSample> samples;   // 按时间升序
  std::optional<double> max_m;
  bool exceeded = false;               // max_m > abs_ref_max_m
};

struct BaseOffsetSample {
  double t = 0.0;
  double offset_m = 0.0;
};

struct BaseStats {
  std::vector<BaseOffsetSample> series;   // 相对时间窗内第一条记录
  std::optional<double> max_m;
  bool exceeded = false;                  // max_m > base_shift_m
};

struct TrackPoint {
  double t = 0.0;
  double e = 0.0, n = 0.0;   // 局部 ENU 东/北(m)
  int q = 0;                 // RTKLIB Q
};

struct Track {
  std::string source;
  std::vector<std::vector<TrackPoint>> segments;   // 按时间断开的折线段
};

struct EventMarker {
  int index = 0;             // 事件表里的编号(从 1 起)
  double e = 0.0, n = 0.0;
};

struct EventCodeSummary {
  std::string code;
  int count = 0;
  double closed_duration_s = 0.0;   // 已关闭事件的时长之和
  int unclosed = 0;
};

struct ReportStats {
  ReportParams params;
  std::vector<SourceStats> sources;              // 顺序同 ordered_source_names
  std::vector<DivergenceStats> divergence;       // 有 rtkrcv 时,can、gpchc 中存在的各一条
  AbsRefStats abs_ref;
  BaseStats base;
  std::vector<ReportEvent> events;
  std::vector<EventCodeSummary> event_summary;   // 次数降序,次数相同按代码
  std::vector<Track> tracks;                     // 顺序同 sources
  std::vector<EventMarker> event_markers;
  std::optional<std::string> track_origin;       // 局部坐标原点取自哪个源("事件" 表示取自事件);无任何位置时为空
  std::vector<std::string> warnings;             // 装载阶段带来的警告
};

// can、gpchc、rtkrcv、ref 在前(存在才列),其余按名字
std::vector<std::string> ordered_source_names(const std::map<std::string, std::vector<PosRecord>>& sources);

// 前置条件:inputs 的记录形如 load_report_inputs() 的输出——每个源的
// PosRecord、events、base_history 都已限定在时间窗内、按时间升序;不做二次校验。
ReportStats compute_report(const ReportInputs& inputs, const ReportParams& params);

}  // namespace gnss_core
