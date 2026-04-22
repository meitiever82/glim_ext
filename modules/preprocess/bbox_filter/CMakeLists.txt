cmake_minimum_required(VERSION 3.16)
project(bbox_filter)

find_package(glim REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)
  
ament_auto_add_library(bbox_filter SHARED
  src/glim_ext/bbox_filter_module.cpp
  src/glim_ext/bbox_filter_module_create.cpp
)
target_include_directories(bbox_filter PRIVATE
  include
)
target_link_libraries(bbox_filter
  glim::glim
  Iridescence::iridescence
  glim_ext
)
