//
// Created by zy on 18-9-5.
//

#include "planning_node.h"

namespace planning {
PlanningNode::PlanningNode(const ros::NodeHandle &nh)
    : nh_(nh) {
   ParamConfig();
   InitROS();
   InitEnv();
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
    sub_map_static_    = nh_.subscribe("/vrep/map_static", 10,
                                       &PlanningNode::CallbackMapStatic, this);
    sub_vehicle_state_ = nh_.subscribe("/vrep/vehicle_state", 10,
                         &PlanningNode::CallbackVehicleState, this);
    pub_trajectory_    = nh_.advertise<nav_msgs::Path>("/trajectory", rate_);
}

void PlanningNode::InitEnv() {
    ros::Rate loop_rate(rate_);
    while (ros::ok()) {
        ros::spinOnce();
        if (static_map_ready_) {
            env_ = new Environment(static_map_, planning_conf_);
            ROS_INFO("[PlanningNode] Environment initialized.");
            break;
        }
    }
}

void PlanningNode::Run() {
    ROS_INFO("[PlanningNode] Planning node begins!");
    ros::Rate loop_rate(rate_);
    while (ros::ok()) {
        ros::spinOnce();
        if (vehicle_state_ready_ && map_ready_) {
            env_->UpdateDynamicMap(map_);
            auto status = rrt_planner_->Solve(vehicle_state_, env_);
            if (status.ok()) {
                ROS_INFO("Solve success.");
            }
            ros::spin();
        }
        loop_rate.sleep();
    }
}

void PlanningNode::RegisterPlanner() {
    rrt_planner_.reset(new HeuristicRRT(planning_conf_.rrt_conf()));
}

void PlanningNode::CallbackMap(const sensor_msgs::Image &msg) {
    map_ = msg;
    map_ready_ = true;
}

void PlanningNode::CallbackMapStatic(const sensor_msgs::Image &msg) {
    static_map_ = msg;
    // ROS_INFO("callback map");
    static_map_ready_ = true;
}

void PlanningNode::CallbackVehicleState(const geometry_msgs::PoseStamped &msg) {
    vehicle_state_.x = msg.pose.position.x;
    vehicle_state_.y = msg.pose.position.y;
    vehicle_state_.theta = tf::getYaw(msg.pose.orientation);
    vehicle_state_ready_ = true;

    // ROS_INFO("callback vehicle state.");
}

}