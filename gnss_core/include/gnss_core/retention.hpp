#pragma once
// 保留天数与磁盘水位清理(spec §3 A6/D4、§5.2)。移植自 rtk-monitor storage/cleanup.py:
// 按日期从旧到新逐个删除,直到"既不超期也不超水位";今天及以后的不删;
// 另加一条:最新的一项永不删(跨天录包时最新目录可能早于今天但仍在写)。
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace gnss_core {

std::optional<int> parse_day_dir_date(const std::string& name);
std::optional<int> parse_bag_dir_date(const std::string& name);
int utc_yyyymmdd(double unix_s);
int days_between(int from_yyyymmdd, int to_yyyymmdd);

struct DatedEntry {
  std::string name;
  int yyyymmdd = 0;
};

std::vector<std::string> sweep_dated_entries(std::vector<DatedEntry> entries, int today_yyyymmdd,
                                             int retention_days, double watermark_pct,
                                             const std::function<double()>& used_pct,
                                             const std::function<bool(const std::string&)>& remove);

struct CleanupReport {
  std::vector<std::string> deleted;
  std::string error;   // 非空表示没能完成(比如 root 不存在)
};

CleanupReport cleanup_dated_root(const std::string& root,
                                 const std::function<std::optional<int>(const std::string&)>& parse_date,
                                 int today_yyyymmdd, int retention_days, double watermark_pct);

}  // namespace gnss_core
