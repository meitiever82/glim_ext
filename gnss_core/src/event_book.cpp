#include "gnss_core/event_book.hpp"

#include <algorithm>
#include <cmath>

namespace gnss_core {

namespace {
void fold_metrics(std::map<std::string, double>& peak, const std::map<std::string, double>& metrics) {
  for (const auto& [key, value] : metrics) {
    const bool is_min = key.size() >= 4 && key.compare(key.size() - 4, 4, "_min") == 0;
    const auto it = peak.find(key);
    if (is_min) {
      if (it == peak.end() || value < it->second) peak[key] = value;
    } else {
      const double current = it == peak.end() ? 0.0 : it->second;
      if (std::abs(value) > std::abs(current)) peak[key] = value;
    }
  }
}
}  // namespace

const char* close_reason_name(CloseReason reason) {
  return reason == CloseReason::Shutdown ? "shutdown" : "recovered";
}

EventBook::EventBook(double close_hysteresis_s) : hysteresis_s_(close_hysteresis_s) {}

EventTransition EventBook::make_close(const std::string& code, const Tracker& tr, double t,
                                      CloseReason reason) {
  EventTransition e;
  e.kind = EventKind::Close;
  e.t = t;
  e.t_open = tr.t_open;
  e.code = code;
  e.level = tr.verdict.level;
  e.message = tr.verdict.message;
  e.pos = tr.last_pos;
  e.reason = reason;
  e.peak = tr.peak;
  return e;
}

std::vector<EventTransition> EventBook::update(double t, const std::vector<Verdict>& verdicts,
                                               std::optional<LatLon> pos,
                                               const std::map<std::string, double>& metrics) {
  std::vector<const Verdict*> active;
  for (const auto& v : verdicts) {
    if (!level_opens_event(v.level)) continue;
    const bool dup = std::any_of(active.begin(), active.end(),
                                 [&](const Verdict* a) { return a->code == v.code; });
    if (!dup) active.push_back(&v);
  }
  const auto is_active = [&](const std::string& code) {
    return std::any_of(active.begin(), active.end(), [&](const Verdict* a) { return a->code == code; });
  };

  std::vector<EventTransition> out;
  for (auto it = open_.begin(); it != open_.end();) {
    Tracker& tr = it->second;
    if (is_active(it->first)) {
      tr.ok_since.reset();
      fold_metrics(tr.peak, metrics);
      if (pos) tr.last_pos = pos;
      ++it;
    } else if (!tr.ok_since) {
      tr.ok_since = t;
      ++it;
    } else if (t - *tr.ok_since >= hysteresis_s_) {
      out.push_back(make_close(it->first, tr, t, CloseReason::Recovered));
      it = open_.erase(it);
    } else {
      ++it;
    }
  }

  for (const Verdict* v : active) {
    if (open_.count(v->code)) continue;
    Tracker tr;
    tr.verdict = *v;
    tr.t_open = t;
    tr.last_pos = pos;
    fold_metrics(tr.peak, metrics);
    open_.emplace(v->code, tr);

    EventTransition e;
    e.kind = EventKind::Open;
    e.t = t;
    e.t_open = t;
    e.code = v->code;
    e.level = v->level;
    e.message = v->message;
    e.pos = pos;
    out.push_back(std::move(e));
  }
  return out;
}

std::vector<EventTransition> EventBook::close_all(double t) {
  std::vector<EventTransition> out;
  for (const auto& [code, tr] : open_) out.push_back(make_close(code, tr, t, CloseReason::Shutdown));
  open_.clear();
  return out;
}

std::vector<std::string> EventBook::open_codes() const {
  std::vector<std::string> codes;
  for (const auto& [code, tr] : open_) codes.push_back(code);
  return codes;
}

}  // namespace gnss_core
