#pragma once
// 诊断引擎:把规则链、偏差监测、事件机、基站监测串起来(spec §8)。
// 输入是带到达时刻(UTC unix 秒)的原始数据,每秒 tick() 一次;落盘、定时、参数加载由壳(轮 3b 的
// gnss_diag_node)负责。语义对应 rtk-monitor main.py 的 _diagnosis_tick 与各 _on_* 回调。
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gnss_core/base_station_monitor.hpp"
#include "gnss_core/diagnosis.hpp"
#include "gnss_core/divergence_monitor.hpp"
#include "gnss_core/event_book.hpp"
#include "gnss_core/rtcm.hpp"
#include "gnss_core/rtkstat.hpp"

namespace gnss_core {

struct BaseUpdate {
  double t = 0.0;
  BaseStationCoords coords;
  BaseFeedResult feed;   // history_changed → 写 base.pos;baseline_learned → 持久化基线
};

struct TickResult {
  DiagnosisResult result;
  std::vector<EventTransition> transitions;
  DivergenceState divergence;
};

// 所有方法的 t / now 参数都要求单调不减(调用方的时钟单调递增);不处理时间倒退——
// 倒退会让新鲜度判断、DivergenceMonitor/EventBook 的迟滞计时等出现未定义的结果。
class DiagnosisEngine {
public:
  // cfg 非法时抛 std::invalid_argument
  DiagnosisEngine(DiagnosisConfig cfg, std::vector<ControlPoint> control_points, bool solver_enabled,
                  std::optional<Ecef> persisted_baseline, std::optional<Ecef> last_history);

  std::vector<BaseUpdate> on_corrections(double t, const uint8_t* data, size_t len);
  void on_solution(double t, const SolutionSample& s);          // rtkrcv 独立解
  void on_device_solution(double t, const SolutionSample& s);   // 610 融合解
  void on_stat_line(double t, const std::string& line);         // rtkrcv $SAT 行

  TickResult tick(double now);
  std::vector<EventTransition> shutdown(double now);
  // 运维确认基站确实搬迁后调用:以最后收到的 1005/1006 坐标为新基线(BaseStationMonitor::reset),
  // held 的基站位移同时清零。返回的 BaseUpdate 供壳持久化基线、写 base.pos(feed = reset 的结果);
  // 还没收到过任何 1005/1006 时什么都不做,返回 nullopt。
  std::optional<BaseUpdate> reset_base_baseline(double t);
  std::optional<Ecef> baseline() const { return base_.baseline(); }
  // 当前已开事件的规则码(字典序),壳发布状态时列出
  std::vector<std::string> open_event_codes() const { return events_.open_codes(); }

private:
  bool fresh(const std::optional<double>& t, double now) const;

  DiagnosisConfig cfg_;
  std::vector<ControlPoint> control_points_;
  bool solver_enabled_;

  RtcmFramer framer_;
  BaseStationMonitor base_;
  DivergenceMonitor divergence_;
  EventBook events_;
  StatEpochAccumulator stat_epoch_;
  SlipWindow slips_;

  std::optional<double> corr_last_t_;
  std::optional<double> base_offset_m_;
  std::optional<BaseStationCoords> last_base_coords_;   // 最后收到的 1005/1006 坐标
  std::optional<SolutionSample> sol_;
  std::optional<double> sol_t_;
  std::optional<SolutionSample> dev_;     // 最新一个 610 解:corr_age 回退与事件位置用
  std::optional<double> dev_t_;
  // 最近 sol_stale_s 内到达的 610 解 (到达时刻, 解),按历元时刻配对时从中挑历元最近的
  std::deque<std::pair<double, SolutionSample>> dev_buf_;
  std::optional<double> stat_t_;
};

}  // namespace gnss_core
