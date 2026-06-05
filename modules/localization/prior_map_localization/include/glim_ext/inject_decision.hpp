#pragma once

namespace glim {

// Stateful hysteresis: enter at >= min_overlap, exit at < 0.8 * min_overlap.
class InjectDecision {
public:
  explicit InjectDecision(double min_overlap) : min_overlap_(min_overlap) {}

  // Update with current overlap; returns whether S2M should be injected this frame.
  bool update(double overlap) {
    if (active_) {
      active_ = overlap >= min_overlap_ * 0.8;
    } else {
      active_ = overlap >= min_overlap_;
    }
    return active_;
  }

  bool active() const { return active_; }
  void reset() { active_ = false; }

private:
  double min_overlap_;
  bool active_ = false;
};

}  // namespace glim
