#pragma once
// 基站坐标监测(spec §3 C2 base_shift、D3 base.pos)。移植自 rtk-monitor diagnosis/base_station.py:
// 首次运行取 warmup_s 内 1005/1006 坐标的各轴中位数作基线,之后报告当前坐标相对基线的 ECEF 位移;
// 坐标变化(任一轴 > 1 mm)时标记 history_changed,由调用方写 base.pos。基线持久化到一个小文件。
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace gnss_core {

struct Ecef {
  double x = 0.0, y = 0.0, z = 0.0;   // m
};

struct BaseFeedResult {
  std::optional<double> offset_m;   // 基线尚未建立时为空
  bool baseline_learned = false;    // 本次调用刚建立/更新了基线(调用方应持久化)
  bool history_changed = false;     // 坐标与上次记录不同(调用方应写 base.pos)
};

class BaseStationMonitor {
public:
  BaseStationMonitor(double warmup_s, std::optional<Ecef> baseline, std::optional<Ecef> last_history);
  BaseFeedResult feed(double t, const Ecef& p);
  // 运维确认基站确实搬迁后,以当前坐标为新基线
  BaseFeedResult reset(const Ecef& p);
  std::optional<Ecef> baseline() const { return baseline_; }

private:
  double warmup_s_;
  std::optional<Ecef> baseline_;
  std::optional<Ecef> last_history_;
  std::vector<std::pair<double, Ecef>> samples_;
};

std::optional<Ecef> read_base_baseline(const std::string& path);
bool write_base_baseline(const std::string& path, const Ecef& p);

}  // namespace gnss_core
