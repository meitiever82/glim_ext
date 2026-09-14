#pragma once
// 诊断事件状态机(spec §3 C10)。每个规则码一个带关闭迟滞的跟踪器,多条同时命中就同时开多个事件。
// 迟滞与峰值指标语义移植自 rtk-monitor diagnosis/events.py;不直接写文件,只返回开/关变更。
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "gnss_core/diagnosis.hpp"

namespace gnss_core {

struct LatLon {
  double lat = 0.0, lon = 0.0;
};

enum class EventKind { Open, Close };
enum class CloseReason { Recovered, Shutdown };
const char* close_reason_name(CloseReason reason);

struct EventTransition {
  EventKind kind = EventKind::Open;
  double t = 0.0;        // 本次变更的时刻
  double t_open = 0.0;   // 事件开启时刻(Open 时等于 t)
  std::string code;
  Level level = Level::Warning;   // 开事件时的结论
  std::string message;            // 开事件时的结论
  std::optional<LatLon> pos;      // Open:开启时位置;Close:事件期间最后一个位置
  CloseReason reason = CloseReason::Recovered;   // 仅 Close 有意义
  std::map<std::string, double> peak;            // 仅 Close 有意义
};

class EventBook {
public:
  explicit EventBook(double close_hysteresis_s);
  std::vector<EventTransition> update(double t, const std::vector<Verdict>& verdicts,
                                      std::optional<LatLon> pos,
                                      const std::map<std::string, double>& metrics);
  // 停机:关闭全部已开事件(reason = Shutdown)
  std::vector<EventTransition> close_all(double t);
  std::vector<std::string> open_codes() const;

private:
  struct Tracker {
    Verdict verdict;
    double t_open = 0.0;
    std::optional<LatLon> last_pos;
    std::optional<double> ok_since;
    std::map<std::string, double> peak;
  };
  static EventTransition make_close(const std::string& code, const Tracker& tr, double t, CloseReason reason);
  double hysteresis_s_;
  std::map<std::string, Tracker> open_;
};

}  // namespace gnss_core
