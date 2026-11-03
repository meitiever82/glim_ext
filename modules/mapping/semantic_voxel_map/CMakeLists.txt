cmake_minimum_required(VERSION 3.22)
project(semantic_voxel_map)

set(CMAKE_CXX_STANDARD 17)

# Built as a sub-directory of the glim_ext package (see glim_ext/CMakeLists.txt,
# ENABLE_SEMANTIC_MAP). Not a standalone colcon package.
find_package(glim REQUIRED)
find_package(spdlog REQUIRED)

add_library(semantic_voxel_map SHARED
  src/glim_ext/semantic_voxel_map_module.cpp
)
target_include_directories(semantic_voxel_map PRIVATE
  include
)
target_link_libraries(semantic_voxel_map
  glim::glim
  glim_ext
  spdlog::spdlog
)
