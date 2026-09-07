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
endif()

if(ament_cmake_FOUND)
  ament_export_targets(gnss_core-targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3 GTSAM GeographicLib)
  ament_package()
else()
  install(EXPORT gnss_core-targets DESTINATION lib/cmake/gnss_core)
endif()
