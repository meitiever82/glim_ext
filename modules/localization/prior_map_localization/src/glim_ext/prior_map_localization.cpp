#include <glim_ext/prior_map_localization.hpp>

#include <algorithm>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/make_shared.h>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim_ext/util/config_ext.hpp>
#include <glim_ext/localization_math.hpp>

namespace glim {

namespace {
// Headroom above smoother_lag so old S2M factors are fully marginalized out before resuming injection.
constexpr double kCooldownMarginSec = 0.5;
}  // namespace

PriorMapLocalization::PriorMapLocalization() : logger_(create_module_logger("prior_loc")) {
  logger_->info("Starting prior_map_localization ...");

  glim::Config config(glim::GlobalConfigExt::get_config_path("config_prior_map_localization"));
  const std::string sec = "prior_map_localization";
  config_.prior_map_path = config.param<std::string>(sec, "prior_map_path", "");
  config_.assume_gravity_aligned = config.param<bool>(sec, "assume_gravity_aligned", true);
  config_.voxel_resolution = config.param<double>(sec, "voxel_resolution", 0.5);
  config_.min_overlap = config.param<double>(sec, "min_overlap", 0.05);
  config_.s2m_points_ratio = config.param<double>(sec, "s2m_points_ratio", 1.0);
  config_.s2m_num_threads = config.param<int>(sec, "s2m_num_threads", 4);
  config_.relocalization_cooldown_sec = config.param<double>(sec, "relocalization_cooldown_sec", 5.0);
  config_.imu_height_above_ground = config.param<double>(sec, "imu_height_above_ground", 0.0);
  config_.ground_grid_resolution = config.param<double>(sec, "ground_grid_resolution", 1.0);

  config_.has_default_init_pose = config.param<bool>(sec, "has_default_init_pose", false);
  if (config_.has_default_init_pose) {
    auto def = config.param<std::vector<double>>(sec, "default_init_pose", std::vector<double>{0, 0, 0, 0});
    if (def.size() == 4) {
      config_.default_init_pose = {def[0], def[1], def[2], def[3]};
    } else {
      logger_->warn("default_init_pose must have 4 elements [x,y,z,yaw]; ignoring");
      config_.has_default_init_pose = false;
    }
  }

  inject_ = InjectDecision(config_.min_overlap);

  // Cooldown lower-bound = smoother_lag + margin (correctness constraint).
  double smoother_lag = 5.0;
  try {
    glim::Config odom_config(glim::GlobalConfig::get_config_path("config_odometry"));
    smoother_lag = odom_config.param<double>("odometry_estimation", "smoother_lag", 5.0);
  } catch (const std::exception& e) {
    logger_->warn("could not read smoother_lag ({}); assuming {:.1f}s", e.what(), smoother_lag);
  }
  effective_cooldown_ = std::max(config_.relocalization_cooldown_sec, smoother_lag + kCooldownMarginSec);
  if (effective_cooldown_ > config_.relocalization_cooldown_sec) {
    logger_->warn("cooldown raised to {:.1f}s to cover smoother_lag {:.1f}s", effective_cooldown_, smoother_lag);
  }

  PriorMapManager::Config mcfg;
  mcfg.voxel_resolution = config_.voxel_resolution;
  mcfg.num_threads = config_.s2m_num_threads;
  mcfg.ground_grid_resolution = config_.ground_grid_resolution;
  map_ = std::make_unique<PriorMapManager>(mcfg);
  if (config_.prior_map_path.empty() || !map_->load_map(config_.prior_map_path)) {
    logger_->error("failed to load prior map: '{}'", config_.prior_map_path);
  } else {
    logger_->info("prior map loaded: {}", config_.prior_map_path);
  }

  if (config_.has_default_init_pose) {
    init_source_.push(config_.default_init_pose[0], config_.default_init_pose[1], config_.default_init_pose[3]);
  }

  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  OdometryEstimationCallbacks::on_new_frame.add(std::bind(&PriorMapLocalization::on_new_frame, this, _1));
  OdometryEstimationCallbacks::on_smoother_update.add(std::bind(&PriorMapLocalization::on_smoother_update, this, _1, _2, _3, _4));
}

PriorMapLocalization::~PriorMapLocalization() {}

void PriorMapLocalization::try_apply_initial_pose(const EstimationFrame::ConstPtr& frame) {
  auto guess = init_source_.pop_pending();
  if (!guess) return;

  const Eigen::Isometry3d X_t = frame->T_world_imu;  // T_odom_imu(t)

  double z = 0.0;
  auto gz = map_->query_ground_height(guess->x, guess->y);
  if (gz) {
    z = *gz + config_.imu_height_above_ground;
  } else {
    z = config_.has_default_init_pose ? config_.default_init_pose[2] : 0.0;
    logger_->warn("no ground height at ({:.1f},{:.1f}); using z={:.2f} (guess outside map?)", guess->x, guess->y, z);
  }

  const Eigen::Isometry3d T_map_imu = compose_initial_pose(guess->x, guess->y, guess->yaw, z, X_t);

  std::lock_guard<std::mutex> lock(state_mutex_);
  const bool was_tracking = (state_ == State::TRACKING);
  T_map_odom_ = compute_T_map_odom(T_map_imu, X_t);
  if (was_tracking) {
    cooldown_until_stamp_ = frame->stamp + effective_cooldown_;  // reloc: cool down
    inject_.reset();
    logger_->info("relocalized; cooling down {:.1f}s", effective_cooldown_);
  }
  state_ = State::TRACKING;
}

void PriorMapLocalization::on_new_frame(const EstimationFrame::ConstPtr& frame) {
  using gtsam::symbol_shorthand::X;

  try_apply_initial_pose(frame);

  Eigen::Isometry3d T_map_odom;
  double cooldown_until;
  bool tracking;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    tracking = (state_ == State::TRACKING);
    T_map_odom = T_map_odom_;
    cooldown_until = cooldown_until_stamp_;
  }
  if (!tracking) {
    if (status_pub_) { std_msgs::msg::String s; s.data = "UNINITIALIZED"; status_pub_->publish(s); }
    return;
  }

  // Overlap (informational + injection gate).
  const Eigen::Isometry3d T_map_sensor = T_map_odom * frame->T_world_imu;
  // map_ is always constructed; compute_overlap_ratio internally returns 0 if the map
  // failed to load (null voxelmap) -> no injection -> pure LIO this frame.
  const double overlap = map_->compute_overlap_ratio(frame->frame, T_map_sensor);
  if (overlap_pub_) {
    std_msgs::msg::Float64 m; m.data = overlap; overlap_pub_->publish(m);
  }

  // Inject S2M factor only when past cooldown AND hysteresis says active.
  // (During cooldown: pure LIO; do not touch hysteresis state.)
  if (frame->stamp >= cooldown_until && inject_.update(overlap)) {
    const gtsam::Pose3 T_odom_map(T_map_odom.inverse().matrix());
    auto src = gtsam_points::random_sampling(frame->frame, config_.s2m_points_ratio, mt_);
    auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(
      T_odom_map, X(frame->id), map_->voxelmap(), src);
    f->set_num_threads(config_.s2m_num_threads);
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_factors_.add(f);
  }

  // --- Publish every tracking frame (T_map_odom constant) ---
  const rclcpp::Time stamp(static_cast<int64_t>(frame->stamp * 1e9));  // RCL_ROS_TIME; ~sub-us precision loss, negligible at 10Hz
  if (tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    const Eigen::Quaterniond q(T_map_odom.rotation());
    tf.transform.translation.x = T_map_odom.translation().x();
    tf.transform.translation.y = T_map_odom.translation().y();
    tf.transform.translation.z = T_map_odom.translation().z();
    tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }
  if (pose_pub_) {
    const Eigen::Isometry3d T_map_imu = T_map_odom * frame->T_world_imu;
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = map_frame_;
    const Eigen::Quaterniond q(T_map_imu.rotation());
    ps.pose.position.x = T_map_imu.translation().x();
    ps.pose.position.y = T_map_imu.translation().y();
    ps.pose.position.z = T_map_imu.translation().z();
    ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
    pose_pub_->publish(ps);
  }
  if (status_pub_) {
    std_msgs::msg::String s; s.data = "TRACKING"; status_pub_->publish(s);
  }
}

void PriorMapLocalization::on_smoother_update(
  gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
  gtsam::NonlinearFactorGraph& new_factors,
  gtsam::Values& new_values,
  std::map<std::uint64_t, double>& new_stamps) {
  (void)smoother;
  (void)new_values;
  (void)new_stamps;
  std::lock_guard<std::mutex> lock(pending_mutex_);
  new_factors.add(pending_factors_);
  pending_factors_.resize(0);
}

std::vector<GenericTopicSubscription::Ptr> PriorMapLocalization::create_subscriptions(rclcpp::Node& node) {
  // ORDERING CONTRACT: GLIM calls create_subscriptions() once on the ROS main thread at
  // startup, strictly BEFORE any odometry callback (on_new_frame) fires. The publisher /
  // broadcaster pointers below are therefore fully constructed before the odometry thread
  // reads them. (Same pattern as glim_ros rviz_viewer.) The null-checks in on_new_frame are
  // a belt-and-suspenders guard, not a substitute for this ordering.
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>("~/localization/pose", 10);
  overlap_pub_ = node.create_publisher<std_msgs::msg::Float64>("~/localization/overlap", 10);
  status_pub_ = node.create_publisher<std_msgs::msg::String>("~/localization/status", 10);

  auto sub = std::make_shared<TopicSubscription<geometry_msgs::msg::PoseWithCovarianceStamped>>(
    "/initialpose",
    [this](const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped>& msg) {
      const auto& q = msg->pose.pose.orientation;
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      init_source_.push(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
      logger_->info("/initialpose received: ({:.2f},{:.2f}, yaw {:.2f})", msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    });
  return {sub};
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::PriorMapLocalization();
}
