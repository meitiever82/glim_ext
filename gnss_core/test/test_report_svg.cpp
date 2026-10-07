#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "gnss_core/report_svg.hpp"
using namespace gnss_core;

namespace {
const double T = 1789430400.0;   // 2026-09-15 00:00:00 UTC
const double kNaN = std::numeric_limits<double>::quiet_NaN();

size_t count(const std::string& hay, const std::string& needle) {
  size_t n = 0;
  for (size_t p = hay.find(needle); p != std::string::npos; p = hay.find(needle, p + needle.size())) ++n;
  return n;
}

// 输出里不能出现 printf 打出来的 nan / inf
void expect_clean(const std::string& svg) {
  EXPECT_EQ(svg.rfind("<svg", 0), 0u) << svg.substr(0, 80);
  EXPECT_EQ(svg.substr(svg.size() - 6), "</svg>");
  EXPECT_EQ(svg.find("nan"), std::string::npos) << svg;
  EXPECT_EQ(svg.find("inf"), std::string::npos) << svg;
  EXPECT_EQ(svg.find("http"), std::string::npos) << "不写 xmlns,报告必须自包含";
}

SvgLineChartOptions window_opts() {
  SvgLineChartOptions o;
  o.t0 = T;
  o.t1 = T + 3600.0;
  o.y_label = "偏差(m)";
  return o;
}
}  // namespace

TEST(ReportSvg, HtmlEscapeCoversMarkupAmpersandAndQuotes) {
  EXPECT_EQ(html_escape("<a href=\"x\">&'</a>"), "&lt;a href=&quot;x&quot;&gt;&amp;&#39;&lt;/a&gt;");
  EXPECT_EQ(html_escape("中文 plain"), "中文 plain");
}

TEST(ReportSvg, FormatUtcShortWithAndWithoutDate) {
  EXPECT_EQ(format_utc_short(T + 3723.9, false), "01:02");
  EXPECT_EQ(format_utc_short(T + 3723.9, true), "09/15 01:02");
}

TEST(ReportSvg, LineChartDrawsSeriesThresholdAndEscapedLabels) {
  auto o = window_opts();
  o.threshold = 0.2;
  o.threshold_label = "阈值 0.2 m";
  const auto svg = svg_line_chart({{"a<b", "#1f77b4", {{T + 10.0, 0.05}, {T + 20.0, 0.1}, {T + 30.0, 0.3}}}}, o);
  expect_clean(svg);
  EXPECT_EQ(count(svg, "<polyline"), 1u);
  EXPECT_NE(svg.find("stroke-dasharray"), std::string::npos) << "阈值画虚线";
  EXPECT_NE(svg.find("阈值 0.2 m"), std::string::npos);
  EXPECT_NE(svg.find("a&lt;b"), std::string::npos);
  EXPECT_EQ(svg.find("a<b"), std::string::npos);
  EXPECT_NE(svg.find("偏差(m)"), std::string::npos);
}

TEST(ReportSvg, LineChartSkipsNonFiniteAndOutOfWindowPointsAndCopesWithFlatData) {
  const auto o = window_opts();
  auto svg = svg_line_chart({{"s", "#000", {{T + 1.0, 0.0}, {T + 2.0, kNaN}, {T + 3.0, 0.0}}}}, o);
  expect_clean(svg);
  EXPECT_EQ(count(svg, "<polyline"), 1u) << "全为 0 的平线也要能画";

  svg = svg_line_chart({{"s", "#000", {{T + 1.0, 0.1}}}}, o);
  expect_clean(svg);
  EXPECT_EQ(count(svg, "<polyline"), 0u);
  EXPECT_EQ(count(svg, "<circle"), 1u) << "单点画圆点";

  EXPECT_NE(svg_line_chart({{"s", "#000", {{T - 100.0, 0.1}, {T + 7200.0, 0.2}}}}, o).find("无数据"), std::string::npos)
      << "窗口外的点不画";
  EXPECT_NE(svg_line_chart({{"s", "#000", {{T + 1.0, kNaN}}}}, o).find("无数据"), std::string::npos);
  EXPECT_NE(svg_line_chart({}, o).find("无数据"), std::string::npos);
  auto bad = o;
  bad.t1 = bad.t0;
  EXPECT_NE(svg_line_chart({{"s", "#000", {{T, 0.1}}}}, bad).find("无数据"), std::string::npos);
}

// 所有 polyline 的 points 属性里的顶点数之和
size_t polyline_vertices(const std::string& svg) {
  size_t n = 0;
  for (size_t p = svg.find("<polyline"); p != std::string::npos; p = svg.find("<polyline", p + 1)) {
    const size_t a = svg.find("points=\"", p) + 8;
    const size_t b = svg.find('"', a);
    n += count(svg.substr(a, b - a), ",");
  }
  return n;
}

// final review Important:31 天 abs-ref 曲线曾把每个样本都写成顶点(41 MB)。
// 每个像素列最多留 min/max 两个点;时间间隔超过 max(60 s, 窗口/绘图宽度×4) 时断开折线。
TEST(ReportSvg, LineChartDownsamplesToPixelColumnsAndBreaksOnTimeGaps) {
  SvgLineChartOptions o;
  o.t0 = T;
  o.t1 = T + 86400.0;
  SvgLineSeries s{"dense", "#1f77b4", {}};
  s.points.reserve(1000000);
  for (int i = 0; i < 1000000; ++i) {
    const double t = T + i * 0.0864;
    if (t >= T + 30000.0 && t < T + 40000.0) continue;   // 两趟之间停了近 3 小时
    s.points.emplace_back(t, i == 123457 ? 5.0 : 0.01 * (i % 7));
  }
  const auto svg = svg_line_chart({s}, o);
  expect_clean(svg);
  EXPECT_LT(svg.size(), 64u * 1024u) << "输出大小与点数无关";
  const size_t plot_columns = 760 - 64 - 16 + 1;
  EXPECT_LE(polyline_vertices(svg), 2u * plot_columns);
  EXPECT_EQ(count(svg, "<polyline"), 2u) << "中间的长间断不能连成一条线";
  EXPECT_NE(svg.find(",24.0"), std::string::npos) << "列内的极值(5.0,顶格 y=24)要保留";
}

TEST(ReportSvg, RatioBarChartSkipsMissingValues) {
  const std::vector<SvgBarGroup> groups{{"00:00", {0.5, std::nullopt}}, {"01:00", {1.0, 0.25}}};
  auto svg = svg_ratio_bar_chart(groups, {"can", "rtkrcv"}, {"#1f77b4", "#ff7f0e"});
  expect_clean(svg);
  EXPECT_EQ(count(svg, "class=\"bar\""), 3u);
  EXPECT_NE(svg.find("01:00"), std::string::npos);
  EXPECT_NE(svg.find("rtkrcv"), std::string::npos);
  svg = svg_ratio_bar_chart({{"00:00", {std::nullopt}}}, {"can"}, {"#1f77b4"});
  EXPECT_NE(svg.find("无数据"), std::string::npos);
}

TEST(ReportSvg, TrackMapDrawsRunsAndMarkersAndCopesWithDegenerateInput) {
  const std::vector<SvgTrackLayer> layers{
      {"can", 3.0, "", {{"#3fb96c", {{0.0, 0.0}, {10.0, 5.0}, {20.0, 5.0}}}, {"#e0b23c", {{20.0, 5.0}, {30.0, 0.0}}}}}};
  auto svg = svg_track_map(layers, {{"1", 10.0, 5.0}, {"<x>", 30.0, 0.0}});
  expect_clean(svg);
  EXPECT_EQ(count(svg, "<polyline"), 2u);
  EXPECT_NE(svg.find(">1</text>"), std::string::npos);
  EXPECT_NE(svg.find("&lt;x&gt;"), std::string::npos);
  EXPECT_NE(svg.find(" m</text>"), std::string::npos) << "比例尺";

  svg = svg_track_map({{"rtkrcv", 2.0, "4 3", {{"#3fb96c", {{5.0, 5.0}}}}}}, {});
  expect_clean(svg);
  EXPECT_EQ(count(svg, "<circle"), 1u) << "单点轨迹画圆点";

  EXPECT_NE(svg_track_map({}, {}).find("无数据"), std::string::npos);
  EXPECT_NE(svg_track_map({{"x", 2.0, "", {{"#000", {{kNaN, 1.0}}}}}}, {}).find("无数据"), std::string::npos);
}
