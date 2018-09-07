//
// Created by zy on 18-9-5.
//

#include "planning_node.h"

namespace planning {
PlanningNode::PlanningNode(const ros::NodeHandle &nh)
    : nh_(nh) {
   ParamConfig();
   InitROS();
   RegisterPlanner();
}

void PlanningNode::ParamConfig() {
    // Get configuration file path.
    std::string planning_path = ros::package::getPath("planning");
    std::string path = planning_path + "/conf/planning_conf.pb.txt";

    // Parse the text file into protobuf.
    using google::protobuf::TextFormat;
    using google::protobuf::io::FileInputStream;
    using google::protobuf::io::ZeroCopyInputStream;
    int file_descriptor = open(path.c_str(), O_RDONLY);
    if (file_descriptor < 0) {
        ROS_ERROR("[PlanningNode] Invalid file descriptor.");
        return;
    }
    ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
    if(!TextFormat::Parse(input, &planning_conf_)) {
        ROS_ERROR("[PlanningNode] Failed to parse file.");
    }
    delete input;
    close(file_descriptor);

    // Print configuration file.
    std::string print_conf;
    TextFormat::PrintToString(planning_conf_, &print_conf);
    ROS_INFO("[PlanningNode] Planning config: \n%s", print_conf.c_str());

    if (planning_conf_.use_sim_time()) {
        ROS_INFO("[PlanningNode] Use sim time.");
        ros::param::set("/use_sim_time", true);
    }

}

void PlanningNode::InitROS() {
    sub_map_           = nh_.subscribe("/vrep/map", 10,
                         &PlanningNode::CallbackMap, this);
    sub_vehicle_state_ = nh_.subscribe("/vrep/vehicle_state", 10,
                         &PlanningNode::CallbackVehicleState, this);
    pub_trajectory_    = nh_.advertise<nav_msgs::Path>("/trajectory", rate_);
}

void PlanningNode::Run() {
    ROS_INFO("[PlanningNode] Planning node begins!");
    ros::Rate loop_rate(rate_);
    while (ros::ok()) {
        ros::spinOnce();
        if (vehicle_state_ready_ && map_ready_) {
            auto status = rrt_planner_->Solve(vehicle_state_, map_);
            if (status.ok()) {
                ROS_INFO("Solve success.");
            }
        }
        loop_rate.sleep();
    }
}

void PlanningNode::RegisterPlanner() {
    rrt_planner_.reset(new HeuristicRRT(planning_conf_.rrt_conf()));
}

void PlanningNode::CallbackMap(const sensor_msgs::Image &msg) {
    map_ = msg;
    // ROS_INFO("callback map");
    map_ready_ = true;
}

void PlanningNode::CallbackVehicleState(const geometry_msgs::PoseStamped &msg) {
    vehicle_state_.x = msg.pose.position.x;
    vehicle_state_.y = msg.pose.position.y;
    vehicle_state_.theta = tf::getYaw(msg.pose.orientation);
    vehicle_state_ready_ = true;

    // ROS_INFO("callback vehicle state.");
}

}