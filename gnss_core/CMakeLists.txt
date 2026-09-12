cmake_minimum_required(VERSION 3.16)
project(gnss_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ament / 纯 CMake 双路径:有 ament(Orin/colcon)走 ament 导出与 ament_add_gtest;
# 无 ament(云端/CI)走纯 CMake + GTest::gtest_main + add_test。同一份源码与测试。
find_package(ament_cmake QUIET)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)            # 4.2 或 4.3 均可,不锁版本
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

add_library(gnss_core SHARED
  src/geodetic.cpp
  src/rtk_fix_buffer.cpp
  src/rtk_noise_policy.cpp
  src/frame_aligner.cpp
  src/trajectory_compare.cpp
  src/pos_io.cpp
  src/synth.cpp
  src/lever_arm_estimator.cpp
  src/anchor_pipeline.cpp
)
target_include_directories(gnss_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GeographicLib_INCLUDE_DIRS}
)
target_link_libraries(gnss_core PUBLIC Eigen3::Eigen gtsam ${GeographicLib_LIBRARIES})

# --- 离线工具(Task 12) ---
add_executable(calibrate_sigma_scale tools/calibrate_sigma_scale.cpp)
target_link_libraries(calibrate_sigma_scale gnss_core)
install(TARGETS calibrate_sigma_scale DESTINATION lib/${PROJECT_NAME})

# --- 合成注入(Task 13) ---
add_executable(synth_rtk_fix tools/synth_rtk_fix.cpp)
target_link_libraries(synth_rtk_fix gnss_core)
install(TARGETS synth_rtk_fix DESTINATION lib/${PROJECT_NAME})
# --- 杆臂/时间偏移估计(Task 14) ---
add_executable(estimate_lever_arm tools/estimate_lever_arm.cpp)
target_link_libraries(estimate_lever_arm gnss_core)
install(TARGETS estimate_lever_arm DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS tools/pos_to_rtkfix_bag.py tools/run_injection_suite.sh tools/traj_rmse.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS gnss_core EXPORT gnss_core-targets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

# --- 测试注册:一个宏,两条路径 ---
function(gnss_core_add_test name)
  if(ament_cmake_FOUND)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${name} test/${name}.cpp)
    target_link_libraries(${name} gnss_core)
  else()
    find_package(GTest REQUIRED)
    add_executable(${name} test/${name}.cpp)
    target_link_libraries(${name} gnss_core GTest::gtest_main)
    add_test(NAME ${name} COMMAND ${name})
  endif()
endfunction()

if(BUILD_TESTING)
  enable_testing()
  gnss_core_add_test(test_types)
  gnss_core_add_test(test_geodetic)
  gnss_core_add_test(test_rtk_fix_buffer)
  gnss_core_add_test(test_rtk_noise_policy)
  gnss_core_add_test(test_frame_aligner)
  gnss_core_add_test(test_antenna_prior_factor)
  gnss_core_add_test(test_pos_io)
  gnss_core_add_test(test_trajectory_compare)
  gnss_core_add_test(test_synth)
  gnss_core_add_test(test_lever_arm_estimator)
  gnss_core_add_test(test_anchor_pipeline)

  # tools/ 下的 Python 工具测试(unittest)。
  # 注意目录是 tests/(与 C++ 的 test/ 分开),必须显式注册,否则这些用例不会被 colcon test 跑到。
  # 用 unittest 而非 pytest:ROS Humble 自带的 launch_testing pytest 插件与新版 pytest(>=8)
  # 不兼容(PluginValidationError: hookimpl 用了已移除的 `path` 参数),而 unittest 不加载任何
  # pytest 插件,因此在装了较新 pytest 的机器上也能跑。测试文件自身用 sys.path 定位 tools/,
  # 与 cwd 无关。
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_export_bag_to_pos
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
            -s ${CMAKE_CURRENT_SOURCE_DIR}/tests -p "test_*.py" -v)
endif()

if(ament_cmake_FOUND)
  ament_export_targets(gnss_core-targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3 GTSAM GeographicLib)
  ament_package()
else()
  install(EXPORT gnss_core-targets DESTINATION lib/cmake/gnss_core)
endif()
