#include "gnss_core/report_svg.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <limits>

namespace gnss_core {

namespace {
std::string fmt(const char* f, double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), f, v);
  return buf;
}

std::string px(double v) { return fmt("%.1f", v); }

std::string svg_open(int w, int h) {
  const std::string ws = std::to_string(w), hs = std::to_string(h);
  return "<svg class=\"chart\" viewBox=\"0 0 " + ws + " " + hs + "\" width=\"" + ws + "\" height=\"" + hs +
         "\" role=\"img\" font-family=\"sans-serif\" font-size=\"11\">";
}

std::string no_data(int w, int h) {
  return svg_open(w, h) + "<text x=\"" + px(w / 2.0) + "\" y=\"" + px(h / 2.0) +
         "\" text-anchor=\"middle\" fill=\"#888\">无数据</text></svg>";
}

// 1/2/5 × 10^k 中不小于 raw 的最小值
double nice_step(double raw) {
  if (!(raw > 0.0) || !std::isfinite(raw)) return 1.0;
  const double mag = std::pow(10.0, std::floor(std::log10(raw)));
  for (double m : {1.0, 2.0, 5.0, 10.0}) {
    if (m * mag >= raw) return m * mag;
  }
  return 10.0 * mag;
}

std::string legend(const std::vector<std::pair<std::string, std::string>>& items, double x, double y) {
  std::string o;
  for (size_t i = 0; i < items.size(); ++i) {
    const double yy = y + 14.0 * static_cast<double>(i);
    o += "<rect x=\"" + px(x) + "\" y=\"" + px(yy - 9.0) + "\" width=\"10\" height=\"10\" fill=\"" +
         html_escape(items[i].second) + "\"/>";
    o += "<text x=\"" + px(x + 14.0) + "\" y=\"" + px(yy) + "\">" + html_escape(items[i].first) + "</text>";
  }
  return o;
}
}  // namespace

std::string html_escape(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += c; break;
    }
  }
  return out;
}

std::string format_utc_short(double unix_s, bool with_date) {
  const std::time_t tt = static_cast<std::time_t>(std::floor(unix_s));
  std::tm tm{};
  gmtime_r(&tt, &tm);
  char buf[32];
  if (with_date) {
    std::snprintf(buf, sizeof(buf), "%02d/%02d %02d:%02d", tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min);
  } else {
    std::snprintf(buf, sizeof(buf), "%02d:%02d", tm.tm_hour, tm.tm_min);
  }
  return buf;
}

std::string svg_line_chart(const std::vector<SvgLineSeries>& series, const SvgLineChartOptions& opt) {
  const int W = opt.width, H = opt.height;
  if (!(opt.t1 > opt.t0) || !std::isfinite(opt.t0) || !std::isfinite(opt.t1)) return no_data(W, H);
  const auto usable = [&](double t, double v) {
    return std::isfinite(t) && std::isfinite(v) && t >= opt.t0 && t <= opt.t1;
  };
  double ymin = 0.0, ymax = 0.0;
  bool any = false;
  for (const auto& s : series) {
    for (const auto& [t, v] : s.points) {
      if (!usable(t, v)) continue;
      any = true;
      ymin = std::min(ymin, v);
      ymax = std::max(ymax, v);
    }
  }
  if (!any) return no_data(W, H);
  const bool has_threshold = opt.threshold && std::isfinite(*opt.threshold);
  if (has_threshold) ymax = std::max(ymax, *opt.threshold);
  const double step = nice_step(ymax - ymin > 0.0 ? (ymax - ymin) / 5.0 : 0.02);
  ymax = std::ceil(ymax / step) * step;
  ymin = std::floor(ymin / step) * step;
  if (ymax <= ymin) ymax = ymin + step;

  const double L = 64.0, R = 16.0, T = 24.0, B = 36.0;
  const auto X = [&](double t) { return L + (t - opt.t0) / (opt.t1 - opt.t0) * (W - L - R); };
  const auto Y = [&](double v) { return H - B - (v - ymin) / (ymax - ymin) * (H - T - B); };

  std::string o = svg_open(W, H);
  for (int i = 0; ymin + step * i <= ymax + step * 0.5; ++i) {
    const double v = ymin + step * i;
    o += "<line x1=\"" + px(L) + "\" y1=\"" + px(Y(v)) + "\" x2=\"" + px(W - R) + "\" y2=\"" + px(Y(v)) +
         "\" stroke=\"#e0e0e0\"/>";
    o += "<text x=\"" + px(L - 6.0) + "\" y=\"" + px(Y(v) + 4.0) + "\" text-anchor=\"end\">" + fmt("%.3f", v) + "</text>";
  }
  const bool with_date = opt.t1 - opt.t0 > 86400.0;
  for (int i = 0; i <= 5; ++i) {
    const double t = opt.t0 + (opt.t1 - opt.t0) * i / 5.0;
    o += "<line x1=\"" + px(X(t)) + "\" y1=\"" + px(H - B) + "\" x2=\"" + px(X(t)) + "\" y2=\"" + px(H - B + 4.0) +
         "\" stroke=\"#444\"/>";
    o += "<text x=\"" + px(X(t)) + "\" y=\"" + px(H - B + 16.0) + "\" text-anchor=\"middle\">" +
         html_escape(format_utc_short(t, with_date)) + "</text>";
  }
  o += "<line x1=\"" + px(L) + "\" y1=\"" + px(H - B) + "\" x2=\"" + px(W - R) + "\" y2=\"" + px(H - B) + "\" stroke=\"#444\"/>";
  o += "<line x1=\"" + px(L) + "\" y1=\"" + px(T) + "\" x2=\"" + px(L) + "\" y2=\"" + px(H - B) + "\" stroke=\"#444\"/>";
  o += "<text x=\"" + px(L) + "\" y=\"" + px(T - 8.0) + "\">" + html_escape(opt.y_label) + "</text>";
  if (has_threshold) {
    const double y = Y(*opt.threshold);
    o += "<line x1=\"" + px(L) + "\" y1=\"" + px(y) + "\" x2=\"" + px(W - R) + "\" y2=\"" + px(y) +
         "\" stroke=\"#d62728\" stroke-dasharray=\"6 4\"/>";
    o += "<text x=\"" + px(W - R) + "\" y=\"" + px(y - 4.0) + "\" text-anchor=\"end\" fill=\"#d62728\">" +
         html_escape(opt.threshold_label) + "</text>";
  }
  // 顶点数与样本数无关(final review:31 天 abs-ref 曲线曾有 41 MB):
  //   - 每个像素列只留该列的最小值点与最大值点(按时间先后),尖峰不会被抽稀掉;
  //   - 相邻样本间隔超过 max(60 s, 每 4 个像素列对应的时长) 时断开折线,分开的两趟不连成一条线。
  // 于是每条序列最多 2×(绘图宽度+1) 个顶点。
  const double plot_w = W - L - R;
  const double gap_s = std::max(60.0, (opt.t1 - opt.t0) / plot_w * 4.0);
  std::vector<std::pair<std::string, std::string>> items;
  for (const auto& s : series) {
    std::vector<std::vector<std::pair<double, double>>> runs;   // 每条折线的 (x, y)
    std::vector<std::pair<double, double>> run;
    long long col = -1;
    std::pair<double, double> col_min, col_max, col_first, col_last;   // (t, v)
    double last_t = 0.0;
    bool have_last = false;
    const auto flush_col = [&]() {
      if (col < 0) return;
      // 列内是平的(最小值点就是最大值点)时改留该列首尾两点,平线才画得出来
      const bool flat = col_min == col_max;
      const auto& a = flat ? col_first : col_min;
      const auto& b = flat ? col_last : col_max;
      const auto& first = a.first <= b.first ? a : b;
      const auto& second = a.first <= b.first ? b : a;
      run.emplace_back(X(first.first), Y(first.second));
      if (second.first != first.first || second.second != first.second) run.emplace_back(X(second.first), Y(second.second));
      col = -1;
    };
    const auto flush_run = [&]() {
      flush_col();
      if (!run.empty()) runs.push_back(std::move(run));
      run.clear();
    };
    for (const auto& [t, v] : s.points) {
      if (!usable(t, v)) continue;
      if (have_last && t - last_t > gap_s) flush_run();
      last_t = t;
      have_last = true;
      const long long c = static_cast<long long>(std::floor(X(t) - L));
      if (c != col) {
        flush_col();
        col = c;
        col_min = col_max = col_first = col_last = {t, v};
      } else {
        col_last = {t, v};
        if (v < col_min.second) col_min = {t, v};
        if (v > col_max.second) col_max = {t, v};
      }
    }
    flush_run();
    if (runs.empty()) continue;
    items.emplace_back(s.label, s.color);
    for (const auto& r : runs) {
      if (r.size() == 1) {
        o += "<circle cx=\"" + px(r[0].first) + "\" cy=\"" + px(r[0].second) + "\" r=\"3\" fill=\"" + html_escape(s.color) +
             "\"/>";
        continue;
      }
      std::string pts;
      for (size_t i = 0; i < r.size(); ++i) pts += (i ? " " : "") + px(r[i].first) + "," + px(r[i].second);
      o += "<polyline fill=\"none\" stroke=\"" + html_escape(s.color) + "\" stroke-width=\"1.5\" points=\"" + pts + "\"/>";
    }
  }
  o += legend(items, W - R - 150.0, T + 4.0);
  o += "</svg>";
  return o;
}

std::string svg_ratio_bar_chart(const std::vector<SvgBarGroup>& groups, const std::vector<std::string>& series_labels,
                                const std::vector<std::string>& colors, int width, int height) {
  const int W = width, H = height;
  bool any = false;
  for (const auto& g : groups) {
    for (const auto& v : g.values) any = any || (v && std::isfinite(*v));
  }
  if (groups.empty() || series_labels.empty() || !any) return no_data(W, H);

  const double L = 48.0, R = 16.0, T = 24.0, B = 40.0;
  const double plot_w = W - L - R, plot_h = H - T - B;
  const double group_w = plot_w / static_cast<double>(groups.size());
  const double bar_w = std::max(1.0, group_w * 0.8 / static_cast<double>(series_labels.size()));
  const auto color = [&](size_t i) { return i < colors.size() ? colors[i] : std::string("#888"); };

  std::string o = svg_open(W, H);
  for (int pct = 0; pct <= 100; pct += 25) {
    const double y = T + plot_h * (1.0 - pct / 100.0);
    o += "<line x1=\"" + px(L) + "\" y1=\"" + px(y) + "\" x2=\"" + px(W - R) + "\" y2=\"" + px(y) + "\" stroke=\"#e0e0e0\"/>";
    o += "<text x=\"" + px(L - 6.0) + "\" y=\"" + px(y + 4.0) + "\" text-anchor=\"end\">" + std::to_string(pct) + "%</text>";
  }
  const size_t label_every = std::max<size_t>(1, (groups.size() + 11) / 12);
  for (size_t gi = 0; gi < groups.size(); ++gi) {
    const double x0 = L + group_w * static_cast<double>(gi) + group_w * 0.1;
    for (size_t si = 0; si < series_labels.size(); ++si) {
      if (si >= groups[gi].values.size()) break;
      const auto& v = groups[gi].values[si];
      if (!v || !std::isfinite(*v)) continue;
      const double r = std::clamp(*v, 0.0, 1.0);
      const double h = r * plot_h;
      o += "<rect class=\"bar\" x=\"" + px(x0 + bar_w * static_cast<double>(si)) + "\" y=\"" + px(T + plot_h - h) +
           "\" width=\"" + px(bar_w) + "\" height=\"" + px(h) + "\" fill=\"" + html_escape(color(si)) + "\"><title>" +
           html_escape(series_labels[si]) + " " + fmt("%.1f%%", r * 100.0) + "</title></rect>";
    }
    if (gi % label_every == 0) {
      o += "<text x=\"" + px(L + group_w * (static_cast<double>(gi) + 0.5)) + "\" y=\"" + px(H - B + 16.0) +
           "\" text-anchor=\"middle\">" + html_escape(groups[gi].label) + "</text>";
    }
  }
  o += "<line x1=\"" + px(L) + "\" y1=\"" + px(T + plot_h) + "\" x2=\"" + px(W - R) + "\" y2=\"" + px(T + plot_h) +
       "\" stroke=\"#444\"/>";
  std::vector<std::pair<std::string, std::string>> items;
  for (size_t i = 0; i < series_labels.size(); ++i) items.emplace_back(series_labels[i], color(i));
  o += legend(items, W - R - 150.0, T + 4.0);
  o += "</svg>";
  return o;
}

std::string svg_track_map(const std::vector<SvgTrackLayer>& layers, const std::vector<SvgMarker>& markers,
                          int width, int height) {
  const int W = width, H = height;
  double min_e = std::numeric_limits<double>::infinity(), max_e = -min_e;
  double min_n = min_e, max_n = -min_e;
  bool any = false;
  const auto grow = [&](double e, double n) {
    if (!std::isfinite(e) || !std::isfinite(n)) return;
    any = true;
    min_e = std::min(min_e, e);
    max_e = std::max(max_e, e);
    min_n = std::min(min_n, n);
    max_n = std::max(max_n, n);
  };
  for (const auto& layer : layers) {
    for (const auto& run : layer.runs) {
      for (const auto& [e, n] : run.en) grow(e, n);
    }
  }
  for (const auto& m : markers) grow(m.e, m.n);
  if (!any) return no_data(W, H);

  const double M = 40.0;
  const double dx = std::max(max_e - min_e, 10.0), dy = std::max(max_n - min_n, 10.0);   // 视野至少 10 m
  const double scale = std::min((W - 2.0 * M) / dx, (H - 2.0 * M) / dy);
  const double ce = (min_e + max_e) / 2.0, cn = (min_n + max_n) / 2.0;
  const auto X = [&](double e) { return W / 2.0 + (e - ce) * scale; };
  const auto Y = [&](double n) { return H / 2.0 - (n - cn) * scale; };

  std::string o = svg_open(W, H);
  o += "<rect x=\"0.5\" y=\"0.5\" width=\"" + px(W - 1.0) + "\" height=\"" + px(H - 1.0) +
       "\" fill=\"#fafafa\" stroke=\"#ccc\"/>";
  for (const auto& layer : layers) {
    const std::string dash = layer.dash.empty() ? "" : " stroke-dasharray=\"" + html_escape(layer.dash) + "\"";
    for (const auto& run : layer.runs) {
      std::string pts;
      size_t n = 0;
      double last_x = 0.0, last_y = 0.0;
      for (const auto& [e, nn] : run.en) {
        if (!std::isfinite(e) || !std::isfinite(nn)) continue;
        last_x = X(e);
        last_y = Y(nn);
        pts += (n++ ? " " : "") + px(last_x) + "," + px(last_y);
      }
      if (n == 0) continue;
      if (n == 1) {
        o += "<circle cx=\"" + px(last_x) + "\" cy=\"" + px(last_y) + "\" r=\"" + px(std::max(2.0, layer.stroke_width)) +
             "\" fill=\"" + html_escape(run.color) + "\"/>";
      } else {
        o += "<polyline fill=\"none\" stroke=\"" + html_escape(run.color) + "\" stroke-width=\"" + px(layer.stroke_width) +
             "\" stroke-linejoin=\"round\"" + dash + " points=\"" + pts + "\"/>";
      }
    }
  }
  for (const auto& m : markers) {
    if (!std::isfinite(m.e) || !std::isfinite(m.n)) continue;
    o += "<g><circle cx=\"" + px(X(m.e)) + "\" cy=\"" + px(Y(m.n)) +
         "\" r=\"8\" fill=\"#fff\" stroke=\"#b00020\" stroke-width=\"1.5\"/>";
    o += "<text x=\"" + px(X(m.e)) + "\" y=\"" + px(Y(m.n) + 3.5) +
         "\" text-anchor=\"middle\" font-size=\"9\" fill=\"#b00020\">" + html_escape(m.text) + "</text></g>";
  }
  const double bar_m = nice_step((W - 2.0 * M) / scale / 5.0);
  const double bar_px = bar_m * scale;
  o += "<line x1=\"" + px(M) + "\" y1=\"" + px(H - 16.0) + "\" x2=\"" + px(M + bar_px) + "\" y2=\"" + px(H - 16.0) +
       "\" stroke=\"#222\" stroke-width=\"2\"/>";
  o += "<text x=\"" + px(M + bar_px + 6.0) + "\" y=\"" + px(H - 12.0) + "\">" + fmt("%g", bar_m) + " m</text>";
  o += "<text x=\"" + px(W - 24.0) + "\" y=\"22\" text-anchor=\"middle\" font-weight=\"bold\">N</text>";
  o += "<line x1=\"" + px(W - 24.0) + "\" y1=\"44\" x2=\"" + px(W - 24.0) + "\" y2=\"26\" stroke=\"#222\" stroke-width=\"2\"/>";
  o += "</svg>";
  return o;
}

}  // namespace gnss_core
