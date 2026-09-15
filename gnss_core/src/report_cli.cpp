#include "gnss_core/report_cli.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <optional>
#include <stdexcept>

#include "gnss_core/pos_io.hpp"
#include "gnss_core/report_html.hpp"
#include "gnss_core/report_inputs.hpp"
#include "gnss_core/report_stats.hpp"
#include "gnss_core/retention.hpp"

namespace gnss_core {

namespace {
const char* const kUsage =
    "用法: gnss_report --root <目录> (--day YYYYMMDD | --from \"YYYY/MM/DD HH:MM:SS\" --to \"YYYY/MM/DD HH:MM:SS\") [选项]\n"
    "  --root DIR                  数据根目录,其下为 YYYYMMDD/ 日期目录(.pos、events.log、base.pos)\n"
    "  --day YYYYMMDD              报告一个 UTC 自然日\n"
    "  --from T --to T             报告 [from, to) 时间窗,UTC,最长 31 天\n"
    "  --out FILE                  输出 HTML 路径(默认 ./gnss_report_<开始>_<结束>.html)\n"
    "  --control-point N,LAT,LON   控制点(十进制度,可重复);不给则不做绝对基准校验\n"
    "  --abs-ref-radius-m M        经过控制点的判定半径,默认 3.0\n"
    "  --abs-ref-max-m M           绝对基准告警阈值,默认 0.2\n"
    "  --base-shift-m M            基站坐标变动告警阈值,默认 0.1\n"
    "  --pair-tol-s S              610 与独立解按时间配对的容差,默认 0.5\n"
    "  --leap-seconds N            .pos 为 GPST 时换算 UTC 的闰秒,默认 18\n"
    "  -h, --help                  显示本说明\n"
    "生成的 HTML 用浏览器打开,\"打印 → 另存为 PDF\" 即得 PDF 报告。\n";

class UsageError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

bool full_double(const std::string& s, double& out) {
  if (s.empty()) return false;
  char* end = nullptr;
  out = std::strtod(s.c_str(), &end);
  return end == s.c_str() + s.size() && std::isfinite(out);
}

double parse_number(const std::string& flag, const std::string& v) {
  double x = 0.0;
  if (!full_double(v, x)) throw UsageError(flag + " 需要一个数字,收到 \"" + v + "\"");
  return x;
}

double parse_positive(const std::string& flag, const std::string& v) {
  const double x = parse_number(flag, v);
  if (!(x > 0.0)) throw UsageError(flag + " 必须大于 0,收到 \"" + v + "\"");
  return x;
}

// parse_utc_date_time 交给 timegm,越界字段会被悄悄规整("2026/02/30" 变成 3 月 2 日);
// 命令行上的时间窗必须是真实存在的日历时刻:月 1–12、日不超过当月天数、时 0–23、分 0–59、秒 [0, 61)。
bool valid_calendar_fields(const std::string& date, const std::string& time) {
  int Y = 0, M = 0, D = 0, h = 0, m = 0, n = 0;
  double sec = 0.0;
  if (std::sscanf(date.c_str(), "%d/%d/%d%n", &Y, &M, &D, &n) != 3 || static_cast<size_t>(n) != date.size()) return false;
  n = 0;
  if (std::sscanf(time.c_str(), "%d:%d:%lf%n", &h, &m, &sec, &n) != 3 || static_cast<size_t>(n) != time.size()) return false;
  static const int kDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (M < 1 || M > 12) return false;
  const bool leap = (Y % 4 == 0 && Y % 100 != 0) || Y % 400 == 0;
  const int month_days = kDays[M - 1] + (M == 2 && leap ? 1 : 0);
  return D >= 1 && D <= month_days && h >= 0 && h <= 23 && m >= 0 && m <= 59 && sec >= 0.0 && sec < 61.0;
}

double parse_time(const std::string& flag, const std::string& v) {
  const size_t sp = v.find(' ');
  double t = 0.0;
  if (sp == std::string::npos || !valid_calendar_fields(v.substr(0, sp), v.substr(sp + 1)) ||
      !parse_utc_date_time(v.substr(0, sp), v.substr(sp + 1), t)) {
    throw UsageError(flag + " 的格式应为 \"YYYY/MM/DD HH:MM:SS\"(UTC),收到 \"" + v + "\"");
  }
  return t;
}

ControlPoint parse_control_point(const std::string& v) {
  const UsageError bad("--control-point 的格式应为 NAME,LAT,LON(十进制度),收到 \"" + v + "\"");
  const size_t a = v.find(',');
  const size_t b = a == std::string::npos ? std::string::npos : v.find(',', a + 1);
  if (a == std::string::npos || a == 0 || b == std::string::npos || v.find(',', b + 1) != std::string::npos) throw bad;
  ControlPoint cp;
  cp.name = v.substr(0, a);
  if (!full_double(v.substr(a + 1, b - a - 1), cp.lat) || !full_double(v.substr(b + 1), cp.lon) ||
      std::abs(cp.lat) > 90.0 || std::abs(cp.lon) > 180.0) {
    throw bad;
  }
  return cp;
}

std::string compact_utc(double t) {
  const std::time_t tt = static_cast<std::time_t>(std::floor(t));
  std::tm tm{};
  gmtime_r(&tt, &tm);
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%04d%02d%02dT%02d%02d%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec);
  return buf;
}

std::string pct(const std::optional<double>& v) {
  if (!v) return "-";
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.1f%%", *v * 100.0);
  return buf;
}
}  // namespace

int run_gnss_report(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
  try {
    std::string root, day, from, to, out_path;
    bool have_from = false, have_to = false;
    ReportParams params;
    PosReadOptions pos_options;
    for (size_t i = 0; i < args.size(); ++i) {
      const std::string& a = args[i];
      const auto value = [&]() -> const std::string& {
        if (i + 1 >= args.size()) throw UsageError(a + " 缺少参数值");
        return args[++i];
      };
      if (a == "-h" || a == "--help") {
        out << kUsage;
        return 0;
      } else if (a == "--root") {
        root = value();
      } else if (a == "--day") {
        day = value();
      } else if (a == "--from") {
        from = value();
        have_from = true;
      } else if (a == "--to") {
        to = value();
        have_to = true;
      } else if (a == "--out") {
        out_path = value();
      } else if (a == "--control-point") {
        params.control_points.push_back(parse_control_point(value()));
      } else if (a == "--abs-ref-radius-m") {
        params.abs_ref_radius_m = parse_positive(a, value());
      } else if (a == "--abs-ref-max-m") {
        params.abs_ref_max_m = parse_positive(a, value());
      } else if (a == "--base-shift-m") {
        params.base_shift_m = parse_positive(a, value());
      } else if (a == "--pair-tol-s") {
        params.pair_tol_s = parse_positive(a, value());
      } else if (a == "--leap-seconds") {
        const std::string& v = value();
        const double x = parse_number(a, v);
        if (x != std::floor(x) || x < 0.0 || x > 100.0) throw UsageError("--leap-seconds 必须是 0 到 100 的整数,收到 \"" + v + "\"");
        pos_options.leap_seconds = static_cast<int>(x);
      } else {
        throw UsageError("不认识的参数: " + a);
      }
    }
    if (root.empty()) throw UsageError("缺少 --root");
    if (!day.empty() && (have_from || have_to)) throw UsageError("--day 与 --from/--to 只能二选一");
    if (day.empty() && !(have_from && have_to)) throw UsageError("需要 --day,或者同时给 --from 与 --to");
    if (!day.empty()) {
      if (!parse_day_dir_date(day) ||
          !valid_calendar_fields(day.substr(0, 4) + "/" + day.substr(4, 2) + "/" + day.substr(6, 2), "00:00:00") ||
          !parse_utc_date_time(day.substr(0, 4) + "/" + day.substr(4, 2) + "/" + day.substr(6, 2), "00:00:00",
                               params.window.t0)) {
        throw UsageError("--day 的格式应为 YYYYMMDD,收到 \"" + day + "\"");
      }
      params.window.t1 = params.window.t0 + 86400.0;
    } else {
      params.window.t0 = parse_time("--from", from);
      params.window.t1 = parse_time("--to", to);
      if (!(params.window.t1 > params.window.t0)) throw UsageError("--to 必须晚于 --from");
      if (params.window.t1 - params.window.t0 > 31.0 * 86400.0) throw UsageError("时间窗最长 31 天");
    }
    if (out_path.empty()) {
      out_path = "gnss_report_" + compact_utc(params.window.t0) + "_" + compact_utc(params.window.t1) + ".html";
    }

    ReportInputs inputs;
    try {
      inputs = load_report_inputs(root, params.window, pos_options);
    } catch (const std::exception& e) {
      err << "gnss_report: " << e.what() << "\n";
      return 2;
    }
    const ReportStats stats = compute_report(inputs, params);
    ReportMeta meta;
    meta.root = root;
    meta.generated_at = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    const std::string html = render_report_html(stats, meta);

    std::ofstream file(out_path, std::ios::binary | std::ios::trunc);
    if (file) {
      file << html;
      file.flush();
    }
    if (!file) {
      err << "gnss_report: 无法写出 " << out_path << "\n";
      return 2;
    }
    for (const auto& w : stats.warnings) err << "gnss_report: 警告: " << w << "\n";
    out << "已生成 " << out_path << "\n";
    for (const auto& s : stats.sources) {
      out << "  " << s.name << ": " << s.epochs << " 历元, 固定率 " << pct(s.fix_ratio) << "\n";
    }
    out << "  事件 " << stats.events.size() << " 条\n";
    return 0;
  } catch (const UsageError& e) {
    err << "gnss_report: " << e.what() << "\n\n" << kUsage;
    return 1;
  }
}

}  // namespace gnss_core
