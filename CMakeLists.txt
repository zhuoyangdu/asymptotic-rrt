cmake_minimum_required(VERSION 2.8.3)
project(planning)
add_definitions(-std=c++11)

include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/planning_conf.proto)
add_library(planning_proto ${PROTO_HDRS} ${PROTO_SRCS})

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

add_library(planning
        src/planner.cpp
        src/rrt/heuristic_rrt.cpp
        src/rrt/heuristic_rrt.h
        src/rrt/node.h
        src/rrt/probablistic_map.cpp
        src/rrt/probablistic_map.h
        src/common/image_proc.cpp
        src/common/environment.cpp
)

target_link_libraries(planning
        ${OpenCV_LIBRARIES}
)

set(EXECUTABLE_OUTPUT_PATH ./bin)

add_executable(planning_node
        src/main.cpp)

target_link_libraries(planning_node
        ${OpenCV_LIBRARIES}
        planning
        planning_proto
        ${PROTOBUF_LIBRARY})


