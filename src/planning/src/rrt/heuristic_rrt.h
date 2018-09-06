//
// Created by zy on 18-9-6.
//

#ifndef PLANNING_RRT_HEURISTIC_RRT_H_
#define PLANNING_RRT_HEURISTIC_RRT_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "planning_conf.pb.h"

namespace planning {

class HeuristicRRT {
 public:
    explicit HeuristicRRT(const RRTConf& rrt_conf);

 private:
    void UniformSample();

    void HeuristicSample();


    RRTConf rrt_conf_;
};

}

#endif //PLANNING_RRT_HEURISTIC_RRT_H_
