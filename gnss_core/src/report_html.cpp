#include "gnss_core/report_html.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gnss_core/diag_io.hpp"
#include "gnss_core/report_svg.hpp"

namespace gnss_core {

namespace {
const char* const kCss = R"CSS(
:root{color-scheme:light}
body{background:#fff;font-family:"Noto Sans CJK SC","Source Han Sans SC","Microsoft YaHei","PingFang SC",sans-serif;color:#222;
  max-width:820px;margin:24px auto;padding:0 16px;font-size:14px;line-height:1.5}
h1{font-size:22px;margin:0 0 12px}
h2{font-size:17px;border-bottom:2px solid #444;padding-bottom:4px;margin-top:30px}
table{border-collapse:collapse;width:100%;margin:8px 0}
th,td{border:1px solid #bbb;padding:3px 6px;text-align:left;vertical-align:top}
th{background:#eee}
td.num{text-align:right;font-variant-numeric:tabular-nums}
tr.bad td{background:#fdecea}
.warn{color:#b00020;font-weight:bold}
.ok{color:#1b7a3a;font-weight:bold}
.note{color:#666;font-size:12px}
.legend span{display:inline-block;margin-right:14px;font-size:12px}
.swatch{display:inline-block;width:12px;height:12px;vertical-align:middle;margin-right:4px}
svg{max-width:100%;height:auto;display:block;margin:8px 0}
@page{size:A4;margin:14mm}
@media print{body{margin:0;max-width:none;font-size:11px}h2{break-after:avoid}svg,tr{break-inside:avoid}thead{display:table-header-group}}
)CSS";

const char* const kSourceColors[] = {"#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd",
                                     "#8c564b", "#e377c2", "#7f7f7f", "#17becf"};

struct TrackStyle {
  double width;
  const char* dash;
  const char* desc;
};
const TrackStyle kTrackStyles[] = {{4.0, "", "粗实线"}, {3.0, "8 4", "虚线"}, {2.0, "", "细实线"}, {1.5, "2 3", "点线"}};

std::string fmt(const char* f, double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), f, v);
  return buf;
}

std::string pct(const std::optional<double>& v) { return v ? fmt("%.1f%%", *v * 100.0) : "-"; }

std::string pct_of(int n, int total) { return total > 0 ? fmt("%.1f%%", 100.0 * n / total) : "-"; }

std::string metres(const std::optional<double>& v) { return v ? fmt("%.3f", *v) : "-"; }

std::string note(const std::string& text) { return "<p class=\"note\">" + text + "</p>"; }

// cells 必须已经转义
std::string table(const std::vector<std::string>& headers, const std::vector<std::vector<std::string>>& rows,
                  const std::vector<std::string>& row_classes = {}) {
  std::string o = "<table><thead><tr>";
  for (const auto& h : headers) o += "<th>" + h + "</th>";
  o += "</tr></thead><tbody>";
  for (size_t i = 0; i < rows.size(); ++i) {
    const std::string cls = i < row_classes.size() && !row_classes[i].empty() ? " class=\"" + row_classes[i] + "\"" : "";
    o += "<tr" + cls + ">";
    for (const auto& c : rows[i]) o += "<td>" + c + "</td>";
    o += "</tr>";
  }
  return o + "</tbody></table>";
}

std::string status(bool bad, const std::string& bad_text) {
  return bad ? "<span class=\"warn\">" + bad_text + "</span>" : "<span class=\"ok\">正常</span>";
}

bool multi_day(const ReportWindow& w) { return w.t1 - w.t0 > 86400.0; }

std::string section_summary(const ReportStats& s, const ReportMeta& m) {
  std::string sources;
  for (const auto& src : s.sources) {
    sources += (sources.empty() ? "" : "、") + html_escape(src.name) + "（" + std::to_string(src.epochs) + " 历元）";
  }
  std::string o = "<section id=\"summary\"><h1>GNSS 定位报告</h1>";
  o += table({"项目", "内容"},
             {{"时间窗", format_utc_timestamp(s.params.window.t0) + " – " + format_utc_timestamp(s.params.window.t1) + "（UTC，含起点不含终点）"},
              {"数据目录", html_escape(m.root)},
              {"生成时间", format_utc_timestamp(m.generated_at) + "（UTC）"},
              {"数据源", sources.empty() ? "无" : sources}});
  if (!s.warnings.empty()) {
    o += "<p class=\"warn\">读取数据时遇到的问题：</p><ul>";
    for (const auto& w : s.warnings) o += "<li>" + html_escape(w) + "</li>";
    o += "</ul>";
  }
  return o + "</section>";
}

std::string section_fix(const ReportStats& s) {
  std::string o = "<section id=\"fix\"><h2>1. 固定解可用率</h2>";
  if (s.sources.empty()) return o + note("时间窗内没有任何 .pos 记录。") + "</section>";
  std::vector<std::vector<std::string>> rows;
  for (const auto& src : s.sources) {
    rows.push_back({html_escape(src.name), std::to_string(src.epochs), "<b>" + pct(src.fix_ratio) + "</b>",
                    pct_of(src.counts.floating, src.epochs), pct_of(src.counts.dgps, src.epochs),
                    pct_of(src.counts.single, src.epochs), pct_of(src.counts.other, src.epochs)});
  }
  o += table({"数据源", "历元数", "固定率", "FLOAT", "DGPS", "SINGLE", "其他"}, rows);
  o += note("固定解按 RTKLIB Q=1 统计；百分比的分母是该源在时间窗内的全部历元。");
  return o + "</section>";
}

// 柱状图超过这么多个小时桶就改为按 UTC 日汇总(Task 4 review 裁定:31 天 744 组柱每组不到 1 px)
constexpr size_t kMaxHourlyBarGroups = 48;

std::string section_hourly(const ReportStats& s) {
  std::string o = "<section id=\"hourly\"><h2>2. 分小时固定率</h2>";
  if (s.sources.empty()) return o + note("时间窗内没有任何 .pos 记录。") + "</section>";
  // 两条规则互相独立:multi_day() 只决定时间标签带不带日期(窗口超过 24 h);
  // kMaxHourlyBarGroups 决定柱状图按小时还是按日(小时桶超过 48 个),表格始终按小时。
  const bool with_date = multi_day(s.params.window);
  std::vector<std::string> labels, colors, headers{"小时（UTC）"};
  for (size_t i = 0; i < s.sources.size(); ++i) {
    labels.push_back(s.sources[i].name);
    colors.push_back(kSourceColors[i % 8]);
    headers.push_back(html_escape(s.sources[i].name) + " 固定率（历元）");
  }
  const size_t hours = s.sources.front().hourly.size();
  const bool daily_bars = hours > kMaxHourlyBarGroups;
  std::vector<SvgBarGroup> groups;
  std::vector<std::vector<std::string>> rows;
  long long current_day = 0;
  std::vector<int> day_fixed, day_epochs;
  const auto flush_day = [&]() {
    if (day_epochs.empty()) return;
    SvgBarGroup g;
    g.label = format_utc_short(static_cast<double>(current_day) * 86400.0, true).substr(0, 5);   // "MM/DD"
    for (size_t i = 0; i < day_epochs.size(); ++i) {
      g.values.push_back(day_epochs[i] > 0 ? std::optional<double>(static_cast<double>(day_fixed[i]) / day_epochs[i])
                                           : std::nullopt);
    }
    groups.push_back(std::move(g));
    day_epochs.clear();
    day_fixed.clear();
  };
  for (size_t h = 0; h < hours; ++h) {
    const double t_start = s.sources.front().hourly[h].t_start;
    const std::string label = format_utc_short(t_start, with_date);
    if (daily_bars) {
      const long long day = static_cast<long long>(std::floor(t_start / 86400.0));
      if (day_epochs.empty() || day != current_day) {
        flush_day();
        current_day = day;
        day_epochs.assign(s.sources.size(), 0);
        day_fixed.assign(s.sources.size(), 0);
      }
    }
    SvgBarGroup g;
    g.label = label;
    std::vector<std::string> row{html_escape(label)};
    bool any = false;
    for (size_t i = 0; i < s.sources.size(); ++i) {
      const HourBucket& b = s.sources[i].hourly[h];
      g.values.push_back(b.fix_ratio);
      row.push_back(b.epochs > 0 ? pct(b.fix_ratio) + "（" + std::to_string(b.epochs) + "）" : "-");
      any = any || b.epochs > 0;
      if (daily_bars) {
        day_epochs[i] += b.epochs;
        day_fixed[i] += b.fixed;
      }
    }
    if (!daily_bars) groups.push_back(std::move(g));
    if (any) rows.push_back(std::move(row));   // 裁定:表格只列有数据的小时
  }
  flush_day();
  o += svg_ratio_bar_chart(groups, labels, colors);
  if (daily_bars) {
    o += note("时间窗超过 " + std::to_string(kMaxHourlyBarGroups) +
              " 小时，柱状图按 UTC 日汇总（当日固定历元 / 当日全部历元）；下表仍按小时。");
  }
  if (rows.empty()) return o + note("时间窗内没有任何历元。") + "</section>";
  o += table(headers, rows);
  o += note("只列出有数据的小时；某个源在该小时没有记录显示“-”。");
  return o + "</section>";
}

std::string section_track(const ReportStats& s) {
  std::string o = "<section id=\"track\"><h2>3. 轨迹与问题路段</h2>";
  if (!s.track_origin) return o + note("时间窗内没有可绘制的位置。") + "</section>";
  std::vector<SvgTrackLayer> layers;
  std::string source_legend;
  for (size_t i = 0; i < s.tracks.size(); ++i) {
    const auto& track = s.tracks[i];
    const TrackStyle& style = kTrackStyles[i % 4];
    SvgTrackLayer layer;
    layer.label = track.source;
    layer.stroke_width = style.width;
    layer.dash = style.dash;
    for (const auto& seg : track.segments) {
      // 同一段内按解状态切成等色子段;切换点同时属于前后两段,折线不断开
      SvgTrackRun run;
      int run_q = seg.empty() ? 0 : seg.front().q;
      for (const auto& pt : seg) {
        if (pt.q != run_q && !run.en.empty()) {
          run.color = quality_color(run_q);
          run.en.emplace_back(pt.e, pt.n);
          layer.runs.push_back(std::move(run));
          run = SvgTrackRun{};
          run_q = pt.q;
        }
        run.en.emplace_back(pt.e, pt.n);
      }
      if (!run.en.empty()) {
        run.color = quality_color(run_q);
        layer.runs.push_back(std::move(run));
      }
    }
    layers.push_back(std::move(layer));
    source_legend += "<span>" + html_escape(track.source) + "：" + style.desc + "</span>";
  }
  std::vector<SvgMarker> markers;
  for (const auto& m : s.event_markers) markers.push_back(SvgMarker{std::to_string(m.index), m.e, m.n});
  o += svg_track_map(layers, markers);
  o += "<div class=\"legend\"><span><i class=\"swatch\" style=\"background:" + quality_color(1) + "\"></i>固定</span>"
       "<span><i class=\"swatch\" style=\"background:" + quality_color(2) + "\"></i>浮点</span>"
       "<span><i class=\"swatch\" style=\"background:" + quality_color(5) + "\"></i>DGPS / 单点</span>"
       "<span><i class=\"swatch\" style=\"background:" + quality_color(0) + "\"></i>其他</span>" +
       source_legend + "</div>";
  const std::string origin = *s.track_origin == "事件" ? "第一个带位置的事件"
                                                        : html_escape(*s.track_origin) + " 的第一条记录";
  o += note("无底图；坐标为以" + origin + "为原点的局部东/北（米），北朝上。红圈数字对应第 7 节事件表的编号，即问题路段位置。");
  for (const auto& track : s.tracks) {
    if (track.gap_s > s.params.track_gap_s) {
      o += note(html_escape(track.source) + " 记录多且断续，为控制图中点数，间隔不超过 " + fmt("%.0f", track.gap_s) +
                " s 的断点已连成一条线（默认 " + fmt("%.0f", s.params.track_gap_s) + " s）。");
    }
  }
  return o + "</section>";
}

std::string section_absref(const ReportStats& s) {
  const auto& p = s.params;
  std::string o = "<section id=\"absref\"><h2>4. 绝对基准校验（控制点比对）</h2>";
  o += "<p>判定：固定解经过控制点 " + fmt("%.1f", p.abs_ref_radius_m) + " m 范围内时，与最近控制点的水平偏差；超过 " +
       fmt("%.3f", p.abs_ref_max_m) + " m 告警。</p>";
  if (p.control_points.empty()) {
    return o + note("未配置控制点（用 --control-point NAME,LAT,LON 指定），本节不做校验。") + "</section>";
  }
  std::string cps;
  for (const auto& cp : p.control_points) {
    cps += (cps.empty() ? "" : "、") + html_escape(cp.name) + "（" + fmt("%.8f", cp.lat) + ", " + fmt("%.8f", cp.lon) + "）";
  }
  o += note("控制点：" + cps);
  if (s.abs_ref.samples.empty()) {
    return o + note("时间窗内没有固定解经过任何控制点 " + fmt("%.1f", p.abs_ref_radius_m) + " m 范围。") + "</section>";
  }
  o += "<p>最大偏差 <b>" + metres(s.abs_ref.max_m) + " m</b>　状态：" +
       status(s.abs_ref.exceeded, "⚠ 超阈值（&gt; " + fmt("%.3f", p.abs_ref_max_m) + " m）——全矿整体平移嫌疑") + "</p>";

  struct Agg {
    int n = 0;
    double max = 0.0, sum = 0.0;
  };
  std::map<std::pair<std::string, std::string>, Agg> agg;
  std::map<std::string, SvgLineSeries> series;
  for (const auto& x : s.abs_ref.samples) {
    auto& a = agg[{x.source, x.control_point}];
    ++a.n;
    a.max = std::max(a.max, x.dev_m);
    a.sum += x.dev_m;
    series[x.source].points.emplace_back(x.t, x.dev_m);
  }
  std::vector<std::vector<std::string>> rows;
  std::vector<SvgLineSeries> chart;
  for (size_t i = 0; i < s.sources.size(); ++i) {
    const std::string& name = s.sources[i].name;
    for (const auto& [key, a] : agg) {
      if (key.first != name) continue;
      rows.push_back({html_escape(name), html_escape(key.second), std::to_string(a.n), fmt("%.3f", a.max),
                      fmt("%.3f", a.sum / a.n)});
    }
    auto it = series.find(name);
    if (it != series.end()) {
      it->second.label = name;
      it->second.color = kSourceColors[i % 8];
      chart.push_back(std::move(it->second));
    }
  }
  o += table({"数据源", "控制点", "经过历元数", "最大偏差（m）", "平均偏差（m）"}, rows);
  SvgLineChartOptions opt;
  opt.y_label = "与控制点的水平偏差（m）";
  opt.t0 = p.window.t0;
  opt.t1 = p.window.t1;
  opt.threshold = p.abs_ref_max_m;
  opt.threshold_label = "告警阈值 " + fmt("%.3f", p.abs_ref_max_m) + " m";
  o += svg_line_chart(chart, opt);
  return o + "</section>";
}

std::string section_divergence(const ReportStats& s) {
  std::string o = "<section id=\"divergence\"><h2>5. 610 与独立解偏差</h2>";
  o += "<p>对每个 rtkrcv 历元取时间最近的 610 历元，时间差 ≤ " + fmt("%.2f", s.params.pair_tol_s) +
       " s 才配对，统计两者的水平距离。</p>";
  if (s.divergence.empty()) {
    return o + note("缺少 rtkrcv.pos，或 can.pos 与 gpchc.pos 都不存在，无法比较。") + "</section>";
  }
  std::vector<std::vector<std::string>> rows;
  for (const auto& d : s.divergence) {
    if (d.n == 0) {
      rows.push_back({html_escape(d.device), html_escape(d.reference), "0", "无配对样本", "-"});
    } else {
      rows.push_back({html_escape(d.device), html_escape(d.reference), std::to_string(d.n), metres(d.max_m), metres(d.mean_m)});
    }
  }
  o += table({"610 数据源", "基准", "配对数", "最大（m）", "均值（m）"}, rows);
  return o + "</section>";
}

std::string section_base(const ReportStats& s) {
  std::string o = "<section id=\"base\"><h2>6. 基站坐标稳定性</h2>";
  if (s.base.series.empty()) return o + note("时间窗内及之前都没有基站坐标记录（base.pos）。") + "</section>";
  const std::string ref_desc =
      s.base.reference_before_window
          ? "时间窗开始前最后一条记录（" + format_utc_timestamp(*s.base.reference_t) + "）"
          : "时间窗内第一条记录（" + format_utc_timestamp(*s.base.reference_t) + "，之前没有更早的记录）";
  o += "<p>相对" + ref_desc + "的最大偏移 <b>" + metres(s.base.max_m) + " m</b>　状态：" +
       status(s.base.exceeded, "⚠ 超过 " + fmt("%.3f", s.params.base_shift_m) + " m——基站坐标可能变动") + "</p>";
  SvgLineSeries series;
  series.label = "基站偏移";
  series.color = "#1f77b4";
  for (const auto& b : s.base.series) series.points.emplace_back(b.t, b.offset_m);
  SvgLineChartOptions opt;
  opt.y_label = "ECEF 偏移（m）";
  opt.t0 = s.params.window.t0;
  opt.t1 = s.params.window.t1;
  opt.threshold = s.params.base_shift_m;
  opt.threshold_label = "告警阈值 " + fmt("%.3f", s.params.base_shift_m) + " m";
  o += svg_line_chart({series}, opt);
  const std::string how = s.base.reference_before_window
                              ? "曲线在时间窗起点处的 0 点即该基准。"
                              : "数据目录里找不到时间窗开始前的记录，只能以时间窗内第一条为基准，窗口开头之前发生的变动看不出来。";
  o += note("时间窗内共 " + std::to_string(s.base.series.size() - (s.base.reference_before_window ? 1 : 0)) +
            " 条记录。base.pos 只在基站坐标变化超过 1 mm 时才记一条，所以基准优先取时间窗开始前的最后一条"
            "（往前逐日查找）；" + how);
  return o + "</section>";
}

std::string section_events(const ReportStats& s) {
  std::string o = "<section id=\"events\"><h2>7. 事件（" + std::to_string(s.events.size()) + " 条）</h2>";
  if (s.events.empty()) return o + note("时间窗内没有诊断事件。") + "</section>";
  std::vector<std::vector<std::string>> summary;
  for (const auto& x : s.event_summary) {
    summary.push_back({html_escape(x.code), std::to_string(x.count), fmt("%.1f", x.closed_duration_s), std::to_string(x.unclosed)});
  }
  o += table({"代码", "次数", "已关闭事件总时长（s）", "未关闭"}, summary);

  std::vector<std::vector<std::string>> rows;
  std::vector<std::string> classes;
  for (size_t i = 0; i < s.events.size(); ++i) {
    const auto& e = s.events[i];
    std::string peak;
    for (const auto& [k, v] : e.peak) peak += (peak.empty() ? "" : "; ") + html_escape(k) + "=" + fmt("%.3f", v);
    const std::string how = e.close_reason == "recovered" ? "恢复" : e.close_reason == "shutdown" ? "停机关闭" : "—";
    rows.push_back({std::to_string(i + 1), format_utc_timestamp(e.t_open),
                    e.t_close ? format_utc_timestamp(*e.t_close) : "未关闭",
                    e.t_close ? fmt("%.1f", *e.t_close - e.t_open) : "-", html_escape(e.level), html_escape(e.code),
                    e.pos ? fmt("%.6f", e.pos->lat) + ", " + fmt("%.6f", e.pos->lon) : "-", html_escape(e.message), how,
                    peak.empty() ? "-" : peak});
    classes.push_back(e.level == "serious" || e.level == "critical" ? "bad" : "");
  }
  o += table({"#", "开始（UTC）", "结束（UTC）", "时长（s）", "级别", "代码", "位置（纬, 经）", "结论", "关闭方式", "峰值"},
             rows, classes);
  o += note("“未关闭”表示数据里没有它的关闭记录：事件在时间窗结束时仍在进行，或诊断节点在关闭前中断。");
  return o + "</section>";
}
}  // namespace

std::string quality_color(int q) {
  switch (q) {
    case 1: return "#3fb96c";
    case 2: return "#e0b23c";
    case 4:
    case 5: return "#e05c4f";
    default: return "#5a6472";
  }
}

std::string render_report_html(const ReportStats& stats, const ReportMeta& meta) {
  std::string o = "<!DOCTYPE html>\n<html lang=\"zh-CN\"><head><meta charset=\"utf-8\"><title>GNSS 定位报告 " +
                  format_utc_timestamp(stats.params.window.t0) + "</title><style>" + kCss + "</style></head><body>";
  o += section_summary(stats, meta);
  o += section_fix(stats);
  o += section_hourly(stats);
  o += section_track(stats);
  o += section_absref(stats);
  o += section_divergence(stats);
  o += section_base(stats);
  o += section_events(stats);
  o += "<footer class=\"note\">由 gnss_core 的 gnss_report 生成。在浏览器中“打印 → 另存为 PDF”即得 PDF 报告。</footer>";
  o += "</body></html>\n";
  return o;
}

}  // namespace gnss_core
