#pragma once
#include <deque>
#include <optional>
#include "gnss_core/types.hpp"

namespace gnss_core {

class RtkFixBuffer {
public:
  void push(const RtkFixSample& s);              // 按 stamp 递增维护
  std::optional<RtkFixSample> interpolate(double t) const;
  void prune(double horizon_s, double now);      // 丢弃 stamp < now - horizon_s
  size_t size() const { return buf_.size(); }
private:
  std::deque<RtkFixSample> buf_;
};

}  // namespace gnss_core
