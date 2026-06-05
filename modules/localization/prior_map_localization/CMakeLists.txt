find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(gtsam_points REQUIRED)

add_library(prior_map_localization SHARED
  src/glim_ext/prior_map_localization.cpp
  src/glim_ext/prior_map_manager.cpp
  src/glim_ext/initial_pose_source.cpp
)
target_include_directories(prior_map_localization PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(prior_map_localization
  glim::glim
  glim_ext
  gtsam_points::gtsam_points
  ${PCL_LIBRARIES}
)
ament_target_dependencies(prior_map_localization
  rclcpp tf2_ros tf2_eigen geometry_msgs sensor_msgs std_msgs
)
target_compile_options(prior_map_localization PRIVATE -std=c++17)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_prior_map_localization
    test/test_prior_map_manager.cpp
    test/test_inject_decision.cpp
    test/test_localization_math.cpp
  )
  target_link_libraries(test_prior_map_localization prior_map_localization)
endif()
