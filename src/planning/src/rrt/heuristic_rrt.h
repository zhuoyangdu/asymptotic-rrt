//
// Created by zy on 18-9-6.
//

#ifndef PLANNING_RRT_HEURISTIC_RRT_H_
#define PLANNING_RRT_HEURISTIC_RRT_H_

#include <iostream>
#include <vector>
#include <memory>

#include <ros/ros.h>
#include <ros/package.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "planning_conf.pb.h"

#include "../common/planning_status.h"
#include "../common/environment.h"

namespace planning {

class HeuristicRRT {
 public:
    HeuristicRRT() {};

    explicit HeuristicRRT(const RRTConf& rrt_conf);

    PlanningStatus Solve(const geometry_msgs::Pose2D& vehicle_state,
                         Environment* environment);

 private:
    PlanningStatus GetGridMap(
        const sensor_msgs::Image& image);

    void UniformSample();

    void HeuristicSample();

    bool is_init_ = false;

    RRTConf rrt_conf_;

};

}

#endif //PLANNING_RRT_HEURISTIC_RRT_H_
