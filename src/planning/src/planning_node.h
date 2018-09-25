// Copyright [2018] <Zhuoyang Du>

#ifndef SRC_PLANNING_SRC_PLANNING_NODE_H_
#define SRC_PLANNING_SRC_PLANNING_NODE_H_

#include <fcntl.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <planning_conf.pb.h>

#include <iostream>

#include "rrt/heuristic_rrt.h"
#include "common/environment.h"

namespace planning {
class PlanningNode {
 public:
    explicit PlanningNode(const ros::NodeHandle& nh);

    void Run();

 private:
    void ParamConfig();

    void InitROS();

    void InitEnv();

    void RegisterPlanner();

    void CallbackMap(const sensor_msgs::Image& msg);
    void CallbackMapStatic(const sensor_msgs::Image& msg);
    void CallbackVehicleState(const geometry_msgs::PoseStamped& msg);

    planning::PlanningConf planning_conf_;
    std::unique_ptr<HeuristicRRT> rrt_planner_;

    uint32_t rate_ = 10;
    ros::NodeHandle nh_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_map_static_;
    ros::Subscriber sub_vehicle_state_;
    ros::Publisher  pub_trajectory_;

    sensor_msgs::Image map_;
    sensor_msgs::Image static_map_;
    geometry_msgs::Pose2D vehicle_state_;
    bool map_ready_ = false;
    bool static_map_ready_ = false;
    bool vehicle_state_ready_ = false;

    Environment* env_;
};
}  // namespace planning

#endif  // SRC_PLANNING_SRC_PLANNING_NODE_H_
