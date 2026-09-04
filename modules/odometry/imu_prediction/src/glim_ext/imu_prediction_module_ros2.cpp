#include <glim_ext/imu_prediction_module.hpp>

#define GLIM_ROS2
#include <glim/util/config.hpp>
#include <glim/util/ros_cloud_converter.hpp>

namespace glim {

std::vector<GenericTopicSubscription::Ptr> IMUPredictionModule::create_subscriptions(rclcpp::Node& node) {
  // deskewed_points_imu_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("~/deskewed_points_imu", rclcpp::SystemDefaultsQoS());
  pred_odom_pub = node.create_publisher<nav_msgs::msg::Odometry>("~/imu_rate_odom", rclcpp::SystemDefaultsQoS());
  pred_pose_pub = node.create_publisher<geometry_msgs::msg::PoseStamped>("~/imu_rate_pose", rclcpp::SystemDefaultsQoS());

  return {};
}

void IMUPredictionModule::publish_pred_frame(const EstimationFrame::ConstPtr& frame) {
  const double max_tol = publish_interval * 10.0;
  // If the next publish time is too far from the current frame's timestamp, reset it to the current frame's timestamp.
  if (std::abs(next_publish_time - frame->stamp) > max_tol || std::abs(last_publish_time - frame->stamp) > max_tol) {
    next_publish_time = frame->stamp;
    last_publish_time = 0.0;
  }

  const double min_publish_interval = publish_interval * 0.25;
  if (next_publish_time > frame->stamp || (frame->stamp - last_publish_time) < min_publish_interval) {
    return;
  }
  next_publish_time += publish_interval;
  last_publish_time = frame->stamp;

  if (imu_frame_id.empty()) {
    imu_frame_id = GlobalConfig::instance()->param<std::string>("meta", "imu_frame_id", "");

    if (imu_frame_id.empty()) {
      logger->warn("IMU frame ID is not set. Using 'imu' as default.");
      imu_frame_id = "imu";
    } else {
      logger->info("auto-detected IMU frame ID: {}", imu_frame_id);
    }
  }

  if (pred_odom_pub->get_subscription_count()) {
    const Eigen::Isometry3d T_odom_imu(frame->T_world_imu);
    const Eigen::Quaterniond quat_odom_imu(T_odom_imu.linear());
    const Eigen::Vector3d v_odom_imu = frame->v_world_imu;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = from_sec(frame->stamp);
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = imu_frame_id;
    odom.pose.pose.position.x = T_odom_imu.translation().x();
    odom.pose.pose.position.y = T_odom_imu.translation().y();
    odom.pose.pose.position.z = T_odom_imu.translation().z();
    odom.pose.pose.orientation.x = quat_odom_imu.x();
    odom.pose.pose.orientation.y = quat_odom_imu.y();
    odom.pose.pose.orientation.z = quat_odom_imu.z();
    odom.pose.pose.orientation.w = quat_odom_imu.w();

    odom.twist.twist.linear.x = v_odom_imu.x();
    odom.twist.twist.linear.y = v_odom_imu.y();
    odom.twist.twist.linear.z = v_odom_imu.z();

    pred_odom_pub->publish(odom);

    logger->debug("published predicted odom (stamp={:.6f})", frame->stamp);
  }

  if (pred_pose_pub->get_subscription_count()) {
    const Eigen::Isometry3d T_odom_imu(frame->T_world_imu);
    const Eigen::Quaterniond quat_odom_imu(T_odom_imu.linear());

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = from_sec(frame->stamp);
    pose.header.frame_id = map_frame_id;
    pose.pose.position.x = T_odom_imu.translation().x();
    pose.pose.position.y = T_odom_imu.translation().y();
    pose.pose.position.z = T_odom_imu.translation().z();
    pose.pose.orientation.x = quat_odom_imu.x();
    pose.pose.orientation.y = quat_odom_imu.y();
    pose.pose.orientation.z = quat_odom_imu.z();
    pose.pose.orientation.w = quat_odom_imu.w();

    pred_pose_pub->publish(pose);

    logger->debug("published predicted pose (stamp={:.6f})", frame->stamp);
  }
}

}  // namespace glim
