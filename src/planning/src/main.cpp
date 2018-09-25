// Copyright [2018] <Zhuoyang Du>

#include <ros/ros.h>

#include "planning_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;
    planning::PlanningNode planning_node(nh);
    planning_node.Run();
    return 0;
}
