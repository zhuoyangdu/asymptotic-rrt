//
// Created by zy on 18-9-5.
//

#include <ros/ros.h>

#include "planning_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;
    planning::PlanningNode planning_node(nh);
    planning_node.Run();
    return 0;
}