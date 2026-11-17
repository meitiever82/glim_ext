# Golden 录制器：独立构建，不进 colcon（本地化工具，只链接 core 源码）
cmake_minimum_required(VERSION 3.16)
project(golden_recorder CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

set(LP ${CMAKE_CURRENT_SOURCE_DIR}/../../localization_parser)

add_executable(golden_recorder
    golden_recorder.cpp
    ${LP}/src/core/common/coordinate.cpp
    ${LP}/src/core/parser/beiyun_parser.cpp
    ${LP}/src/core/parser/huace_parser.cpp
    ${LP}/src/core/processor/beiyun_processor.cpp
    ${LP}/src/core/processor/huace_processor.cpp
    ${LP}/src/core/processor/processor_base.cpp
    ${LP}/src/core/transport/data_recorder.cpp
)
target_include_directories(golden_recorder PRIVATE ${LP}/include)
target_link_libraries(golden_recorder PRIVATE yaml-cpp Eigen3::Eigen)

# parity 回放器：只依赖 gnss_core 的 judge（纯函数）+ yaml
set(GNSS_CORE ${CMAKE_CURRENT_SOURCE_DIR}/../../../gnss_core)
add_executable(golden_replay
    golden_replay.cpp
    ${GNSS_CORE}/src/judge.cpp
)
target_include_directories(golden_replay PRIVATE ${GNSS_CORE}/include)
target_link_libraries(golden_replay PRIVATE yaml-cpp)
