cmake_minimum_required(VERSION 2.8.3)
project(planning)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    roslib
)

include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/planning_conf.proto)
add_library(planning_proto ${PROTO_HDRS} ${PROTO_SRCS})

catkin_package(
    LIBRARIES planning
    CATKIN_DEPENDS roscpp rospy roslib std_msgs geometry_msgs sensor_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_library(planning
        src/planning_node.cpp
        src/planning_node.h
        src/rrt/heuristic_rrt.cpp
        src/rrt/heuristic_rrt.h
        src/rrt/tree.cpp
        src/rrt/tree.h
        src/rrt/node.h
)

target_link_libraries(planning
        ${catkin_LIBRARIES}
)

add_executable(planning_node
        src/main.cpp)

target_link_libraries(planning_node
        ${catkin_LIBRARIES}
        planning
        planning_proto
        ${PROTOBUF_LIBRARY})