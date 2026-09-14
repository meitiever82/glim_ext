#include "gnss_core/retention.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <ctime>
#include <filesystem>

namespace gnss_core {

namespace {
// Howard Hinnant 的 days_from_civil:公历日期 → 自 1970-01-01 起的天数
long long days_from_civil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const long long era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + static_cast<long long>(doe) - 719468;
}

bool all_digits(const std::string& s) {
  return !s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isdigit(c); });
}

std::optional<int> parse_yyyymmdd(const std::string& s) {
  if (s.size() != 8 || !all_digits(s)) return std::nullopt;
  const int v = std::stoi(s);
  const int y = v / 10000, m = (v / 100) % 100, d = v % 100;
  if (m < 1 || m > 12 || d < 1) return std::nullopt;
  static const int kDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  const bool leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
  const int max_day = kDays[m - 1] + (m == 2 && leap ? 1 : 0);
  if (d > max_day) return std::nullopt;
  return v;
}

long long to_days(int yyyymmdd) {
  return days_from_civil(yyyymmdd / 10000, static_cast<unsigned>((yyyymmdd / 100) % 100),
                         static_cast<unsigned>(yyyymmdd % 100));
}
}  // namespace

std::optional<int> parse_day_dir_date(const std::string& name) { return parse_yyyymmdd(name); }

std::optional<int> parse_bag_dir_date(const std::string& name) {
  // gnss_YYYYMMDD_HHMMSS
  if (name.size() != 20 || name.compare(0, 5, "gnss_") != 0 || name[13] != '_') return std::nullopt;
  if (!all_digits(name.substr(14, 6))) return std::nullopt;
  return parse_yyyymmdd(name.substr(5, 8));
}

int utc_yyyymmdd(double unix_s) {
  const std::time_t tt = static_cast<std::time_t>(std::floor(unix_s));
  std::tm tm{};
  gmtime_r(&tt, &tm);
  return (tm.tm_year + 1900) * 10000 + (tm.tm_mon + 1) * 100 + tm.tm_mday;
}

int days_between(int from_yyyymmdd, int to_yyyymmdd) {
  return static_cast<int>(to_days(to_yyyymmdd) - to_days(from_yyyymmdd));
}

std::vector<std::string> sweep_dated_entries(std::vector<DatedEntry> entries, int today_yyyymmdd,
                                             int retention_days, double watermark_pct,
                                             const std::function<double()>& used_pct,
                                             const std::function<bool(const std::string&)>& remove) {
  std::sort(entries.begin(), entries.end(), [](const DatedEntry& a, const DatedEntry& b) {
    return a.yyyymmdd != b.yyyymmdd ? a.yyyymmdd < b.yyyymmdd : a.name < b.name;
  });
  std::vector<std::string> deleted;
  for (size_t i = 0; i + 1 < entries.size(); ++i) {   // 最新一项永不删
    const DatedEntry& e = entries[i];
    if (e.yyyymmdd >= today_yyyymmdd) break;
    const bool over = used_pct() > watermark_pct;
    const bool too_old = days_between(e.yyyymmdd, today_yyyymmdd) > retention_days;
    if (!(too_old || over)) break;
    if (!remove(e.name)) break;
    deleted.push_back(e.name);
  }
  return deleted;
}

CleanupReport cleanup_dated_root(const std::string& root,
                                 const std::function<std::optional<int>(const std::string&)>& parse_date,
                                 int today_yyyymmdd, int retention_days, double watermark_pct) {
  namespace fs = std::filesystem;
  CleanupReport report;
  std::error_code ec;
  std::vector<DatedEntry> entries;
  fs::directory_iterator it(root, ec);
  if (ec) {
    report.error = "无法遍历 " + root + ": " + ec.message();
    return report;
  }
  for (const auto& entry : it) {
    std::error_code type_ec;
    if (!entry.is_directory(type_ec) || type_ec) continue;
    const std::string name = entry.path().filename().string();
    if (const auto date = parse_date(name)) entries.push_back({name, *date});
  }
  const auto used_pct = [&root]() {
    std::error_code space_ec;
    const auto info = fs::space(root, space_ec);
    if (space_ec || info.capacity == 0) return 0.0;   // 查不到用量时只按保留天数删
    return static_cast<double>(info.capacity - info.free) / static_cast<double>(info.capacity) * 100.0;
  };
  const auto remove = [&root](const std::string& name) {
    std::error_code rm_ec;
    fs::remove_all(fs::path(root) / name, rm_ec);
    return !rm_ec;
  };
  report.deleted = sweep_dated_entries(std::move(entries), today_yyyymmdd, retention_days, watermark_pct,
                                       used_pct, remove);
  return report;
}

}  // namespace gnss_core
