cmake_minimum_required(VERSION 3.22)
project(rtk_odometry)

set(CMAKE_CXX_STANDARD 17)

# Built as a sub-directory of the glim_ext package (see glim_ext/CMakeLists.txt,
# ENABLE_RTK_ODOMETRY). Not a standalone colcon package.
find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(spdlog REQUIRED)
find_package(rclcpp REQUIRED)
# gnss_core's config re-finds GeographicLib; its Find module is not on the default path.
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(gnss_core REQUIRED)
find_package(gnss_msgs REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(rtk_odometry SHARED
  src/glim_ext/rtk_odometry_module_ros2.cpp
)
target_include_directories(rtk_odometry PRIVATE
  include
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
)
target_link_libraries(rtk_odometry
  glim_ext
  gnss_core::gnss_core
  ${GTSAM_LIBRARIES}
  ${glim_LIBRARIES}
  spdlog::spdlog
)
ament_target_dependencies(rtk_odometry rclcpp gnss_msgs)
