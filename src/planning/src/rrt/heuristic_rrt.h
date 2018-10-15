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
#include "node.h"
#include "tree.h"
#include "probablistic_map.h"

namespace planning {

class HeuristicRRT {
 public:
    HeuristicRRT() = default;

    explicit HeuristicRRT(const RRTConf& rrt_conf);

    PlanningStatus Solve(const geometry_msgs::Pose2D& vehicle_state,
                         Environment* environment);

 private:
    PlanningStatus GetGridMap(
        const sensor_msgs::Image& image);

    struct Compare {
        Compare(Node sample) {this->sample = sample;}
        bool operator() (Node& a, Node& b) {
            int dist1 = (a.col() - sample.col()) * (a.col() - sample.col())
                      + (a.row() - sample.row()) * (a.row() - sample.row());
            int dist2 = (b.col() - sample.col()) * (b.col() - sample.col())
                      + (b.row() - sample.row()) * (b.row() - sample.row());
            return dist1 > dist2;
        }
       Node sample;
    };

    bool GetNearestNode(const Node& sample,
                        const std::vector<Node> tree,
                        Node* nearest_node);

    bool CheckCollision(const Node& a, const Node& b, const Environment& env);

    void Steer(const Node& sample, const Node& nearest, Node* new_node);

    ros::NodeHandle private_nh_;
    ros::Publisher pub_map_;

    bool is_init_ = false;
    RRTConf rrt_conf_;
    bool show_image_ = false;
};

}  // namespace planning

#endif  // SRC_PLANNING_SRC_RRT_HEURISTIC_RRT_H_
