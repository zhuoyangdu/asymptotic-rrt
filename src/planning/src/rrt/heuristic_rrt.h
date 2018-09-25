// Copyright [2018] <Zhuoyang Du>

#ifndef SRC_PLANNING_SRC_RRT_HEURISTIC_RRT_H_
#define SRC_PLANNING_SRC_RRT_HEURISTIC_RRT_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <planning_conf.pb.h>

#include <iostream>
#include <vector>
#include <memory>

#include <grid_map_ros/grid_map_ros.hpp>

#include "../common/planning_status.h"
#include "../common/environment.h"
#include "../common/image_proc.h"

namespace planning {

class HeuristicRRT {
 public:
    HeuristicRRT() = default;

    explicit HeuristicRRT(const RRTConf& rrt_conf);

    PlanningStatus Solve(const geometry_msgs::Pose2D& vehicle_state,
                         Environment* environment);

 private:
    ros::NodeHandle private_nh_;
    ros::Publisher pub_map_;

    PlanningStatus GetGridMap(
        const sensor_msgs::Image& image);

    void UniformSample();

    void HeuristicSample();

    bool is_init_ = false;

    RRTConf rrt_conf_;
};

}  // namespace planning

#endif  // SRC_PLANNING_SRC_RRT_HEURISTIC_RRT_H_
