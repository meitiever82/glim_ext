#include <glim_ext/initial_pose_source.hpp>

namespace glim {

void InitialPoseSource::push(double x, double y, double yaw) {
  std::lock_guard<std::mutex> lock(mutex_);
  pending_ = Pose2D{x, y, yaw};
}

std::optional<InitialPoseSource::Pose2D> InitialPoseSource::pop_pending() {
  std::lock_guard<std::mutex> lock(mutex_);
  auto p = pending_;
  pending_.reset();
  return p;
}

}  // namespace glim
