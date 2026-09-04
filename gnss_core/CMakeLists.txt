cmake_minimum_required(VERSION 3.8)
project(gnss_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

add_library(gnss_core SHARED
  src/geodetic.cpp
  src/rtk_fix_buffer.cpp
  src/rtk_noise_policy.cpp
  src/frame_aligner.cpp
  src/trajectory_compare.cpp
  src/pos_io.cpp
)
target_include_directories(gnss_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GeographicLib_INCLUDE_DIRS}
)
target_link_libraries(gnss_core Eigen3::Eigen gtsam ${GeographicLib_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS gnss_core EXPORT gnss_core-targets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(gnss_core-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 GTSAM)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_types test/test_types.cpp)
  target_link_libraries(test_types gnss_core)
endif()

ament_package()
