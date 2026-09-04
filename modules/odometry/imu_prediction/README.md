# imu_prediction_module

## About

This module predicts the IMU pose and publishes it at the rate of IMU data. Because the prediction is based on the last known pose and velocity, it may involve some (negligible) drift over time between point cloud updates.

## Topics

- ~/imu_rate_odom (nav_msgs/msg/Odometry): Predicted IMU pose and velocity in the odom frame (contains estimation drift).
- ~/imu_rate_pose (geometry_msgs/msg/PoseStamped): Predicted IMU pose in the map frame (drift-corrected).
