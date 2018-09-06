//
// Created by zy on 18-9-5.
//

#ifndef PLANNING_PLANNING_NODE_H
#define PLANNING_PLANNING_NODE_H

#include <iostream>
#include <fcntl.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "planning_conf.pb.h"

#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

namespace planning {
class PlanningNode {
 public:
    PlanningNode(const ros::NodeHandle& nh);

    void Run();

 private:
    void ParamConfig();

    void InitROS();

    void CallbackMap(const sensor_msgs::Image& msg);
    void CallbackVehicleState(const geometry_msgs::PoseStamped& msg);

    planning::PlanningConf planning_conf_;

    uint32_t rate_ = 10;
    ros::NodeHandle nh_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_vehicle_state_;
    ros::Publisher  pub_trajectory_;

    sensor_msgs::Image map_;
    geometry_msgs::PoseStamped vehicle_state_;
    bool map_ready_ = false;
    bool vehicle_state_ready_ = false;
};
}

#endif //PLANNING_PLANNING_NODE_H
