cmake_minimum_required(VERSION 3.20)
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
  src/process_supervisor.cpp
  src/port_probe.cpp
  src/executable_lookup.cpp
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

add_executable(rtkrcv_node src/rtkrcv_node.cpp)
target_link_libraries(rtkrcv_node gnss_bringup_io gnss_core::gnss_core)
ament_target_dependencies(rtkrcv_node rclcpp gnss_msgs)
install(TARGETS rtkrcv_node DESTINATION lib/${PROJECT_NAME})

add_executable(pos_writer src/pos_writer_node.cpp)
target_link_libraries(pos_writer gnss_bringup_io gnss_core::gnss_core)
ament_target_dependencies(pos_writer rclcpp gnss_msgs)
install(TARGETS pos_writer DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tcp_stream test/test_tcp_stream.cpp)
  # final-fix-wave 第 2 项:test_tcp_stream.cpp 用 dlsym(RTLD_NEXT, "socket")
  # 拦截 socket() 调用来抓 run_server() 的监听 fd(见测试文件里的注释),
  # 和 test_local_reserver 拦截 close()/accept4()/socket()/pipe2() 是同一个
  # 手法,同样需要显式链接 dl。
  target_link_libraries(test_tcp_stream gnss_bringup_io dl)

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

  ament_add_gtest(test_process_supervisor test/test_process_supervisor.cpp)
  target_link_libraries(test_process_supervisor gnss_bringup_io)
  target_compile_definitions(test_process_supervisor PRIVATE
    FAKE_RTKRCV_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/fake_rtkrcv.sh")

  # resolve_executable:ProcessSupervisor 在父进程里按 PATH 解析 binary(execv
  # 不查 PATH,子进程又已 chdir 到 run_dir,裸名字 "rtkrcv" 以前永远起不来)。
  ament_add_gtest(test_executable_lookup test/test_executable_lookup.cpp)
  target_link_libraries(test_executable_lookup gnss_bringup_io)

  # rtk_fix_mapping.hpp 是纯函数(不碰 ROS 节点),含 LineSplitter 这个供
  # rtkrcv_node 按行切分 TcpStream 字节流用的纯逻辑,单独可测。
  ament_add_gtest(test_rtk_fix_mapping test/test_rtk_fix_mapping.cpp)
  target_link_libraries(test_rtk_fix_mapping gnss_core::gnss_core)
  target_include_directories(test_rtk_fix_mapping PRIVATE include)
  ament_target_dependencies(test_rtk_fix_mapping gnss_msgs)

  # port_probe 用 LocalReserver 自己起一个监听来验证探测结果,和进程监管
  # 无关,单独成文件(task-1-brief.md 的要求)。
  ament_add_gtest(test_port_probe test/test_port_probe.cpp)
  target_link_libraries(test_port_probe gnss_bringup_io)

  # pos_rotation.hpp 是纯函数(按天轮转的目录/文件路径决策),不碰 ROS 节点
  # 或真实文件系统,和 rtk_fix_mapping.hpp 里的 plan_stat_tail 同样的处理
  # 方式,单独可测。
  ament_add_gtest(test_pos_rotation test/test_pos_rotation.cpp)
  target_include_directories(test_pos_rotation PRIVATE include)

  # pos_writer_params.hpp 是纯函数(time_system/leap_seconds/record 时间戳
  # 合理性的校验),不碰 ROS 节点,pos_writer_node.cpp 自己的可独立测试的
  # 部分,和 rtcm_bridge_params.hpp/test_rtcm_bridge_params 同样的处理方式。
  # 头文件里用到 gnss_core::PosTimeSystem,需要链接 gnss_core 取得头文件的
  # include 路径(与 test_rtk_fix_mapping 同样的道理)。
  ament_add_gtest(test_pos_writer_params test/test_pos_writer_params.cpp)
  target_link_libraries(test_pos_writer_params gnss_core::gnss_core)
  target_include_directories(test_pos_writer_params PRIVATE include)

  # round 2 review 的 Important 4:pos_writer_node.cpp 里"一路 .pos 输出"的
  # 状态机(PosDecimator+PosWriter+当前路径+失败锁存+沉默检测)抽成
  # PosSourceWriter,不碰任何 ROS 类型,可以完全脱离节点单测——两个此前能
  # 骗过全部 142 个测试的变异(轮转依据换成"现在"、失败锁存改坏)现在都会
  # 被这里的用例直接抓到。
  ament_add_gtest(test_pos_source_writer test/test_pos_source_writer.cpp)
  target_link_libraries(test_pos_source_writer gnss_core::gnss_core)
  target_include_directories(test_pos_source_writer PRIVATE include)
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# record_gnss.sh:rosbag2 录制脚本(spec §5.2),装到 lib/${PROJECT_NAME}
# 而不是 share,好让 `ros2 run gnss_bringup record_gnss.sh` 可以像
# rtcm_bridge/rtkrcv_node/pos_writer 一样直接按包名+文件名找到它
# (`ros2 run` 只在 lib/<package> 下查找可执行文件)。
install(PROGRAMS scripts/record_gnss.sh DESTINATION lib/${PROJECT_NAME})

ament_export_targets(gnss_bringup-targets HAS_LIBRARY_TARGET)
ament_package()
