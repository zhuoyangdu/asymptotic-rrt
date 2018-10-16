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
    cv_bridge
    grid_map_ros
)

include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/planning_conf.proto)
add_library(planning_proto ${PROTO_HDRS} ${PROTO_SRCS})

find_package(OpenCV REQUIRED)

catkin_package(
    LIBRARIES planning
    CATKIN_DEPENDS roscpp rospy roslib std_msgs geometry_msgs sensor_msgs cv_bridge grid_map_ros
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(planning
        src/planning_node.cpp
        src/planning_node.h
        src/rrt/heuristic_rrt.cpp
        src/rrt/heuristic_rrt.h
        src/rrt/node.h
        src/rrt/probablistic_map.cpp
        src/rrt/probablistic_map.h
        src/common/image_proc.cpp
        src/common/environment.cpp
)

target_link_libraries(planning
        ${catkin_LIBRARIES}
)

add_executable(planning_node
        src/main.cpp)

target_link_libraries(planning_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        planning
        planning_proto
        ${PROTOBUF_LIBRARY})

add_executable(test_image_proc
        src/test/test_image_proc.cpp)

target_link_libraries(test_image_proc
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        planning
        planning_proto
        ${PROTOBUF_LIBRARY})
