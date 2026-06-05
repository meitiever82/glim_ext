#pragma once

#include <mutex>
#include <random>
#include <memory>
#include <array>
#include <map>
#include <cstdint>

#include <Eigen/Geometry>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <glim/util/extension_module_ros2.hpp>
#include <glim/odometry/estimation_frame.hpp>

#include <glim_ext/prior_map_manager.hpp>
#include <glim_ext/inject_decision.hpp>
#include <glim_ext/initial_pose_source.hpp>

namespace glim {

class PriorMapLocalization : public ExtensionModuleROS2 {
public:
  PriorMapLocalization();
  ~PriorMapLocalization() override;

  std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;

private:
  enum class State { UNINITIALIZED, TRACKING };

  void on_new_frame(const EstimationFrame::ConstPtr& frame);
  void on_smoother_update(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    gtsam::NonlinearFactorGraph& new_factors,
    gtsam::Values& new_values,
    std::map<std::uint64_t, double>& new_stamps);

  void try_apply_initial_pose(const EstimationFrame::ConstPtr& frame);

  struct Config {
    std::string prior_map_path;
    bool assume_gravity_aligned = true;
    double voxel_resolution = 0.5;
    double min_overlap = 0.05;
    double s2m_points_ratio = 1.0;
    int s2m_num_threads = 4;
    double relocalization_cooldown_sec = 5.0;
    std::array<double, 4> default_init_pose = {0, 0, 0, 0};
    bool has_default_init_pose = false;
    double imu_height_above_ground = 0.0;
    double ground_grid_resolution = 1.0;
  };
  Config config_;
  double effective_cooldown_ = 5.0;

  std::unique_ptr<PriorMapManager> map_;
  InitialPoseSource init_source_;
  InjectDecision inject_{0.05};

  std::mutex state_mutex_;
  State state_ = State::UNINITIALIZED;
  Eigen::Isometry3d T_map_odom_ = Eigen::Isometry3d::Identity();
  double cooldown_until_stamp_ = -1.0;
  std::mt19937 mt_{12345};

  gtsam::NonlinearFactorGraph pending_factors_;
  std::mutex pending_mutex_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr overlap_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";

  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace glim
