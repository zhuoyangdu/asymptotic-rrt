//
// Created by zy on 18-9-5.
//

#include "planning_node.h"

namespace planning {
PlanningNode::PlanningNode(const ros::NodeHandle &nh)
    : nh_(nh) {
   ParamConfig();
}

void PlanningNode::ParamConfig() {

}

void PlanningNode::InitROS() {
    sub_map_ = nh_.subscribe(
        "/vrep/map", 10, &PlanningNode::CallbackMap, this);
    sub_vehicle_state_ = nh_.subscribe("/vrep/vehicle_state", 10, &PlanningNode::CallbackVehicleState, this);
    pub_trajectory_ = nh_.advertise<nav_msgs::Path>("/trajectory", rate_);
}

void PlanningNode::Run() {
    ROS_INFO("[PlanningNode] Planning node begins!");
}

void PlanningNode::CallbackMap(const sensor_msgs::Image &msg) {
    map_ = msg;
    map_ready_ = true;
}

void PlanningNode::CallbackVehicleState(const geometry_msgs::PoseStamped &msg) {
    vehicle_state_ = msg;
    vehicle_state_ready_ = true;
}

}