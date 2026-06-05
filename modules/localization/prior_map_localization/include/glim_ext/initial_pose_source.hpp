#pragma once

#include <mutex>
#include <optional>

namespace glim {

// Holds the latest pending 2D pose guess (x, y, yaw in map). Thread-safe.
class InitialPoseSource {
public:
  struct Pose2D { double x, y, yaw; };

  void push(double x, double y, double yaw);
  std::optional<Pose2D> pop_pending();

private:
  std::mutex mutex_;
  std::optional<Pose2D> pending_;
};

}  // namespace glim
