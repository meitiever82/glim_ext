#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "gnss_core/report_html.hpp"
using namespace gnss_core;

namespace {
const double T = 1789430400.0;   // 2026-09-15 00:00:00 UTC
const double kLat = 44.5, kLon = 90.28;

double north(double m) { return kLat + m / 111132.0; }

PosRecord rec(double t, int q, double lat = kLat) {
  PosRecord r;
  r.stamp = t;
  r.q = q;
  r.lat = lat;
  r.lon = kLon;
  r.height = 600.0;
  return r;
}

ReportEvent event(const std::string& code, double t_open, std::optional<double> t_close, const std::string& reason,
                  const std::string& message = "结论") {
  ReportEvent e;
  e.code = code;
  e.level = "serious";
  e.message = message;
  e.t_open = t_open;
  e.t_close = t_close;
  e.close_reason = reason;
  e.pos = LatLon{kLat, kLon};
  return e;
}

ReportParams day_params() {
  ReportParams p;
  p.window = ReportWindow{T, T + 86400.0};
  return p;
}

ReportInputs rich_inputs() {
  ReportInputs in;
  for (int i = 0; i < 20; ++i) in.sources["can"].push_back(rec(T + i, i < 10 ? 1 : 2, north(i)));
  for (int i = 0; i < 20; ++i) in.sources["rtkrcv"].push_back(rec(T + i, 1, north(i + 0.3)));
  in.events = {event("corr_outage", T + 1.0, T + 11.0, "recovered"), event("low_sats", T + 5.0, T + 6.0, "shutdown"),
               event("multipath", T + 8.0, std::nullopt, "")};
  in.base_history = {BaseSample{T, Ecef{-2148744.0, 4426641.0, 4044655.0}},
                     BaseSample{T + 5.0, Ecef{-2148744.02, 4426641.0, 4044655.0}}};
  return in;
}

std::string section(const std::string& html, const std::string& id, const std::string& next_id) {
  const size_t a = html.find("id=\"" + id + "\"");
  const size_t b = html.find("id=\"" + next_id + "\"");
  if (a == std::string::npos || b == std::string::npos || b < a) return "";
  return html.substr(a, b - a);
}

size_t count(const std::string& hay, const std::string& needle) {
  size_t n = 0;
  for (size_t p = hay.find(needle); p != std::string::npos; p = hay.find(needle, p + needle.size())) ++n;
  return n;
}
}  // namespace

TEST(ReportHtml, QualityColorsMatchRtkMonitor) {
  EXPECT_EQ(quality_color(1), "#3fb96c");
  EXPECT_EQ(quality_color(2), "#e0b23c");
  EXPECT_EQ(quality_color(4), "#e05c4f");
  EXPECT_EQ(quality_color(5), "#e05c4f");
  EXPECT_EQ(quality_color(0), "#5a6472");
}

TEST(ReportHtml, ContainsEverySectionInOrderAndIsSelfContained) {
  auto p = day_params();
  p.control_points = {ControlPoint{"K1", kLat, kLon}};
  const auto html = render_report_html(compute_report(rich_inputs(), p), ReportMeta{"/data/gnss/pos", T + 90000.0});
  EXPECT_EQ(html.rfind("<!DOCTYPE html>", 0), 0u);
  size_t last = 0;
  for (const char* id : {"summary", "fix", "hourly", "track", "absref", "divergence", "base", "events"}) {
    const size_t at = html.find(std::string("id=\"") + id + "\"");
    ASSERT_NE(at, std::string::npos) << id;
    EXPECT_GT(at, last) << id << " 顺序不对";
    last = at;
  }
  EXPECT_NE(html.find("@page"), std::string::npos);
  EXPECT_NE(html.find("@media print"), std::string::npos);
  EXPECT_NE(html.find(":root{color-scheme:light}"), std::string::npos) << "暗色浏览器里也按浅色渲染";
  EXPECT_NE(html.find("background:#fff"), std::string::npos) << "body 显式白底";
  EXPECT_EQ(html.find("<script"), std::string::npos);
  EXPECT_EQ(html.find("<link"), std::string::npos);
  EXPECT_EQ(html.find("http://"), std::string::npos);
  EXPECT_EQ(html.find("https://"), std::string::npos);
  EXPECT_EQ(html.find("nan"), std::string::npos);
  EXPECT_GE(count(html, "<svg"), 4u) << "分小时柱状图、轨迹图、绝对基准曲线、基站曲线";
  EXPECT_NE(html.find("2026/09/15 00:00:00.000"), std::string::npos) << "时间窗按 UTC 显示";
  EXPECT_NE(section(html, "fix", "hourly").find("50.0%"), std::string::npos) << "can 固定率 10/20";
  EXPECT_NE(section(html, "fix", "hourly").find("100.0%"), std::string::npos) << "rtkrcv 固定率 20/20";
}

TEST(ReportHtml, EscapesEverythingThatComesFromFilesOrTheCommandLine) {
  ReportInputs in = rich_inputs();
  in.sources["evil<i>"] = {rec(T + 1.0, 1)};
  in.events.push_back(event("<b>code</b>", T + 2.0, T + 3.0, "recovered", "<script>alert(1)</script>"));
  auto p = day_params();
  p.control_points = {ControlPoint{"K&1", kLat, kLon}};
  const auto html = render_report_html(compute_report(in, p), ReportMeta{"/data/a&b", T});
  EXPECT_EQ(html.find("<script>alert"), std::string::npos);
  EXPECT_NE(html.find("&lt;script&gt;alert(1)&lt;/script&gt;"), std::string::npos);
  EXPECT_EQ(html.find("<b>code</b>"), std::string::npos);
  EXPECT_NE(html.find("&lt;b&gt;code&lt;/b&gt;"), std::string::npos);
  EXPECT_EQ(html.find("evil<i>"), std::string::npos);
  EXPECT_NE(html.find("evil&lt;i&gt;"), std::string::npos);
  EXPECT_NE(html.find("K&amp;1"), std::string::npos);
  EXPECT_NE(html.find("/data/a&amp;b"), std::string::npos);
}

TEST(ReportHtml, StatusLinesFollowTheThresholds) {
  ReportInputs in = rich_inputs();
  auto p = day_params();
  p.control_points = {ControlPoint{"K1", north(5.0), kLon}};   // rtkrcv 在 T+5 附近 0.3 m 处经过
  p.abs_ref_max_m = 0.2;
  p.base_shift_m = 0.01;
  auto html = render_report_html(compute_report(in, p), ReportMeta{"/r", T});
  EXPECT_NE(section(html, "absref", "divergence").find("全矿整体平移嫌疑"), std::string::npos);
  EXPECT_NE(section(html, "base", "events").find("基站坐标可能变动"), std::string::npos);

  p.abs_ref_max_m = 5.0;
  p.base_shift_m = 1.0;
  html = render_report_html(compute_report(in, p), ReportMeta{"/r", T});
  EXPECT_NE(section(html, "absref", "divergence").find("正常"), std::string::npos);
  EXPECT_EQ(section(html, "absref", "divergence").find("全矿整体平移嫌疑"), std::string::npos);
  EXPECT_NE(section(html, "base", "events").find("正常"), std::string::npos);
}

TEST(ReportHtml, EventTableShowsNumbersCloseReasonsAndUnclosedEvents) {
  const auto html = render_report_html(compute_report(rich_inputs(), day_params()), ReportMeta{"/r", T});
  const std::string ev = html.substr(html.find("id=\"events\""));
  EXPECT_NE(ev.find("恢复"), std::string::npos);
  EXPECT_NE(ev.find("停机关闭"), std::string::npos);
  EXPECT_NE(ev.find("未关闭"), std::string::npos);
  EXPECT_NE(ev.find("<td>3</td>"), std::string::npos) << "事件编号与轨迹图标注一致";
  EXPECT_NE(ev.find("44.500000, 90.280000"), std::string::npos) << "问题路段的经纬度";
  EXPECT_NE(ev.find("10.0"), std::string::npos) << "corr_outage 时长";
  const std::string track = section(html, "track", "absref");
  EXPECT_NE(track.find(">3</text>"), std::string::npos) << "轨迹图上标出事件编号";
}

TEST(ReportHtml, TrackRunsAreColouredByQuality) {
  const auto html = render_report_html(compute_report(rich_inputs(), day_params()), ReportMeta{"/r", T});
  const std::string track = section(html, "track", "absref");
  EXPECT_NE(track.find("stroke=\"#3fb96c\""), std::string::npos) << "固定解段";
  EXPECT_NE(track.find("stroke=\"#e0b23c\""), std::string::npos) << "can 后半段是浮点解";
}

// 表格 tbody 里的行数
size_t body_rows(const std::string& html_part) {
  const size_t a = html_part.find("<tbody>");
  const size_t b = html_part.find("</tbody>");
  if (a == std::string::npos || b == std::string::npos) return 0;
  return count(html_part.substr(a, b - a), "<tr");
}

// 裁定(Task 4 review):超过 48 个小时桶时柱状图改为按 UTC 日的固定率(sum fixed / sum epochs),表格仍按小时
TEST(ReportHtml, MultiDayWindowsChartDailyFixRatiosButKeepTheHourlyTable) {
  ReportInputs in;
  for (int h = 0; h < 31 * 24; ++h) in.sources["can"].push_back(rec(T + h * 3600.0 + 10.0, h % 4 == 0 ? 1 : 2));
  ReportParams p;
  p.window = ReportWindow{T, T + 31 * 86400.0};
  const auto html = render_report_html(compute_report(in, p), ReportMeta{"/r", T});
  const std::string hourly = section(html, "hourly", "track");
  const size_t svg_end = hourly.find("</svg>");
  ASSERT_NE(svg_end, std::string::npos);
  const std::string chart = hourly.substr(0, svg_end);
  EXPECT_LE(count(chart, "<rect class=\"bar\""), 31u) << "31 天最多 31 组柱";
  EXPECT_GE(count(chart, "<rect class=\"bar\""), 31u) << "每天都有数据";
  EXPECT_NE(chart.find("can 25.0%"), std::string::npos) << "日固定率 = 当天固定历元 / 当天历元";
  EXPECT_NE(hourly.find("按 UTC 日"), std::string::npos) << "要说明柱状图的口径变了";
  EXPECT_EQ(body_rows(hourly), 31u * 24u) << "表格仍按小时";

  // 48 个小时桶以内仍按小时画
  ReportParams two_days;
  two_days.window = ReportWindow{T, T + 2 * 86400.0};
  const std::string h2 = section(render_report_html(compute_report(in, two_days), ReportMeta{"/r", T}), "hourly", "track");
  EXPECT_EQ(count(h2.substr(0, h2.find("</svg>")), "<rect class=\"bar\""), 48u);
}

// 裁定(final review):表格只列有数据的小时,并注明;柱状图不变
TEST(ReportHtml, HourlyTableListsOnlyHoursWithData) {
  ReportInputs in;
  in.sources["can"] = {rec(T + 3600.0 + 1.0, 1), rec(T + 3600.0 + 2.0, 2)};
  in.sources["rtkrcv"] = {rec(T + 3 * 3600.0 + 1.0, 1)};
  const auto html = render_report_html(compute_report(in, day_params()), ReportMeta{"/r", T});
  const std::string hourly = section(html, "hourly", "track");
  EXPECT_EQ(body_rows(hourly), 2u) << "01 时(can)与 03 时(rtkrcv)";
  EXPECT_NE(hourly.find("<td>01:00</td>"), std::string::npos);
  EXPECT_NE(hourly.find("<td>03:00</td>"), std::string::npos);
  EXPECT_EQ(hourly.find("<td>02:00</td>"), std::string::npos);
  EXPECT_NE(hourly.find("只列出有数据的小时"), std::string::npos);
}

// 所有 polyline 的顶点数之和
size_t polyline_vertices(const std::string& svg) {
  size_t n = 0;
  for (size_t p = svg.find("<polyline"); p != std::string::npos; p = svg.find("<polyline", p + 1)) {
    const size_t a = svg.find("points=\"", p) + 8;
    n += count(svg.substr(a, svg.find('"', a) - a), ",");
  }
  return n;
}

// final review 压力回归:31 天窗口(评审实测 4 源 1 Hz 生成 41 MB HTML)。单元测试里用轻一些但同形态的数据:
// 1 个源、5 s 一条、每 10 条夹一个 11 s 的断点(> track_gap_s,约 5 万段),轨迹是半径 500 m、
// 每小时一圈、经过控制点的环线(每天 23 趟经过),每天第一个小时停在控制点旁(abs-ref 样本约 2 万),
// 每 20 分钟有 2 分钟浮点解,每天 4 条事件、4 条基站记录。
// 上限的依据:轨迹 ≤ 4000 点、两条曲线各 ≤ 2×681 顶点、31 组日柱,分小时表 744 行(约 60 KB)、
// 事件表 124 行(约 50 KB)——都与样本数无关;1 MB 给这些固定开销留出数倍余量,
// 同时比修复前小一个数量级以上(修复前同一数据:HTML 6.1 MB、轨迹 95658 点、abs-ref 曲线 20336 顶点、744 组柱;修复后约 256 KB)。
TEST(ReportHtml, ThirtyOneDayWindowStaysBoundedInSizeAndPointCounts) {
  const double r_m = 500.0;
  const double m_per_deg_lon = 111320.0 * std::cos(kLat * M_PI / 180.0);
  ReportInputs in;
  auto& can = in.sources["can"];
  double t = T;
  for (int i = 0; t < T + 31 * 86400.0; ++i) {
    const double theta = 2.0 * M_PI * std::fmod(t - T, 3600.0) / 3600.0;
    PosRecord r = rec(t, std::fmod(t - T, 1200.0) < 120.0 ? 2 : 1, north(r_m * (1.0 - std::cos(theta))));
    r.lon = kLon + r_m * std::sin(theta) / m_per_deg_lon;
    if (std::fmod(t - T, 86400.0) < 3600.0) {   // 每天第一个小时停在控制点旁(abs-ref 样本密集)
      r = rec(t, 1, north(0.5 * std::sin(t)));
    }
    can.push_back(r);
    t += (i % 10 == 9) ? 11.0 : 5.0;
  }
  for (int d = 0; d < 31; ++d) {
    for (int k = 0; k < 4; ++k) {
      const double t_open = T + d * 86400.0 + k * 21600.0 + 100.0;
      in.events.push_back(event("corr_outage", t_open, t_open + 30.0, "recovered"));
      in.base_history.push_back(BaseSample{t_open, Ecef{-2148744.0 + 1e-3 * k, 4426641.0, 4044655.0}});
    }
  }
  ReportParams p;
  p.window = ReportWindow{T, T + 31 * 86400.0};
  p.control_points = {ControlPoint{"K1", kLat, kLon}};
  const auto stats = compute_report(in, p);
  ASSERT_GT(stats.abs_ref.samples.size(), 10000u) << "abs-ref 样本要远多于曲线的顶点上限,才测得出抽稀";
  size_t track_points = 0;
  for (const auto& seg : stats.tracks.at(0).segments) track_points += seg.size();
  EXPECT_LE(track_points, p.max_track_points) << can.size() << " 条记录";

  const auto html = render_report_html(stats, ReportMeta{"/r", T});
  EXPECT_LT(html.size(), 1024u * 1024u) << "can " << can.size() << " 条, abs-ref " << stats.abs_ref.samples.size();
  const std::string absref = section(html, "absref", "divergence");
  EXPECT_LE(polyline_vertices(absref), 2u * (760u - 64u - 16u + 1u));
  const std::string base = section(html, "base", "events");
  EXPECT_LE(polyline_vertices(base), 2u * (760u - 64u - 16u + 1u));
  const std::string hourly = section(html, "hourly", "track");
  EXPECT_EQ(count(hourly.substr(0, hourly.find("</svg>")), "<rect class=\"bar\""), 31u);
  std::printf("[ stress   ] can %zu 条, abs-ref %zu 样本, 轨迹 %zu 点, HTML %zu 字节\n", can.size(),
              stats.abs_ref.samples.size(), track_points, html.size());
}

TEST(ReportHtml, EmptyInputsRenderPlaceholdersInsteadOfFailing) {
  const auto html = render_report_html(compute_report(ReportInputs{}, day_params()), ReportMeta{"/r", T});
  for (const char* text : {"时间窗内没有任何 .pos 记录", "没有可绘制的位置", "未配置控制点", "无法比较",
                           "没有基站坐标记录", "时间窗内没有诊断事件"}) {
    EXPECT_NE(html.find(text), std::string::npos) << text;
  }
  EXPECT_EQ(html.find("nan"), std::string::npos);
}
