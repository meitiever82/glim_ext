#pragma once
// 报告里的内嵌 SVG 图表(spec §3 F2,轮 4a)。纯字符串生成,不依赖图表库;
// 非有限数值与时间窗外的点一律跳过,输出里不会出现 nan / inf;不写 xmlns(HTML5 内联不需要)。
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace gnss_core {

// & < > " ' 转义,文本与属性值通用
std::string html_escape(const std::string& s);

// UTC 时刻短格式:"HH:MM";with_date 时 "MM/DD HH:MM"
std::string format_utc_short(double unix_s, bool with_date);

struct SvgLineSeries {
  std::string label;
  std::string color;                               // CSS 颜色
  std::vector<std::pair<double, double>> points;   // (UTC unix 秒, 值),时间升序
};

struct SvgLineChartOptions {
  std::string y_label;
  double t0 = 0.0, t1 = 0.0;          // 横轴 = 报告时间窗
  std::optional<double> threshold;    // 画一条红色虚线
  std::string threshold_label;
  int width = 760, height = 260;
};

std::string svg_line_chart(const std::vector<SvgLineSeries>& series, const SvgLineChartOptions& opt);

struct SvgBarGroup {
  std::string label;                            // 横轴分组标签
  std::vector<std::optional<double>> values;    // 与 series_labels 一一对应,取值 [0,1];空 = 无数据,不画
};

std::string svg_ratio_bar_chart(const std::vector<SvgBarGroup>& groups, const std::vector<std::string>& series_labels,
                                const std::vector<std::string>& colors, int width = 760, int height = 260);

struct SvgTrackRun {                             // 同一解质量、颜色相同的一段折线
  std::string color;
  std::vector<std::pair<double, double>> en;    // (东, 北) m
};

struct SvgTrackLayer {                           // 一个数据源
  std::string label;
  double stroke_width = 2.0;
  std::string dash;                              // stroke-dasharray;空 = 实线
  std::vector<SvgTrackRun> runs;
};

struct SvgMarker {                               // 事件位置标注
  std::string text;
  double e = 0.0, n = 0.0;
};

// 等比例的东/北平面图,北朝上,带比例尺与北向标;无底图
std::string svg_track_map(const std::vector<SvgTrackLayer>& layers, const std::vector<SvgMarker>& markers,
                          int width = 760, int height = 560);

}  // namespace gnss_core
