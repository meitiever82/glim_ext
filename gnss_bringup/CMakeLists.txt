cmake_minimum_required(VERSION 3.16)
project(gnss_bringup)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gnss_msgs REQUIRED)
# gnss_core 的 ament_cmake_export_dependencies-extras.cmake 会重新 find_package(GeographicLib),
# 但不会带上 gnss_core 自己 CMakeLists.txt 里追加的 CMAKE_MODULE_PATH,
# 下游包(本包)必须自己补上,否则找不到 FindGeographicLib.cmake。
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(gnss_core REQUIRED)

# 不依赖 ROS 的 I/O 与进程管理,便于单测
add_library(gnss_bringup_io SHARED
  src/tcp_stream.cpp
  src/local_reserver.cpp
  src/rtkrcv_conf.cpp
)
target_include_directories(gnss_bringup_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gnss_bringup_io pthread)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS gnss_bringup_io EXPORT gnss_bringup-targets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

add_executable(rtcm_bridge src/rtcm_bridge_node.cpp)
target_link_libraries(rtcm_bridge gnss_bringup_io)
ament_target_dependencies(rtcm_bridge rclcpp gnss_msgs)
install(TARGETS rtcm_bridge DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tcp_stream test/test_tcp_stream.cpp)
  target_link_libraries(test_tcp_stream gnss_bringup_io)

  ament_add_gtest(test_local_reserver test/test_local_reserver.cpp)
  # test_local_reserver 用 dlsym(RTLD_NEXT, ...) 重新定义
  # close()/accept4()/socket()/pipe2() 来追踪 fd 生命周期(见测试文件里
  # close_tracker 的注释),需要显式链接 dl。
  target_link_libraries(test_local_reserver gnss_bringup_io dl)

  # rtcm_bridge_node.cpp 自己的纯函数逻辑(端口合法性、streams 去重检测)
  ament_add_gtest(test_rtcm_bridge_params test/test_rtcm_bridge_params.cpp)
  target_include_directories(test_rtcm_bridge_params PRIVATE include)

  ament_add_gtest(test_rtkrcv_conf test/test_rtkrcv_conf.cpp)
  target_link_libraries(test_rtkrcv_conf gnss_bringup_io)
endif()

ament_export_targets(gnss_bringup-targets HAS_LIBRARY_TARGET)
ament_package()
