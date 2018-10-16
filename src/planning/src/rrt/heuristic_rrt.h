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

#include "../common/planning_status.h"
#include "../common/environment.h"
#include "../common/image_proc.h"
#include "node.h"
#include "probablistic_map.h"

using namespace std;

namespace planning {

class HeuristicRRT {
 public:
    HeuristicRRT() = default;

    explicit HeuristicRRT(const PlanningConf& planning_conf);

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

    bool Steer(const Node& sample, const Node& nearest, Node* new_node);

    bool CheckTarget(const Node& node, const Node& goal);

    vector<Node> GetPath(const std::vector<Node>& tree, const Node& new_node);

    double PathLength(const std::vector<Node>& path);

    std::vector<Node> PostProcessing(const std::vector<Node>& path,
                                     const Environment* env);

    void Record(const std::vector<Node>& tree);

    ros::NodeHandle private_nh_;
    ros::Publisher pub_map_;

    bool is_init_ = false;
    PlanningConf planning_conf_;
    RRTConf rrt_conf_;
    bool show_image_ = false;
    double shortest_path_length_ = 0;
    std::vector<Node> min_path;
};

}  // namespace planning

#endif  // SRC_PLANNING_SRC_RRT_HEURISTIC_RRT_H_
