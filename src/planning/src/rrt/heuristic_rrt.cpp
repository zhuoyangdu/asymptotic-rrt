//
// Created by zy on 18-9-6.
//

#include "heuristic_rrt.h"

namespace planning {

HeuristicRRT::HeuristicRRT(const RRTConf& rrt_conf)
        : rrt_conf_(rrt_conf), is_init_(true) {

}

PlanningStatus HeuristicRRT::Solve(
    const geometry_msgs::Pose2D& vehicle_state,
    const sensor_msgs::Image& map) {
    return PlanningStatus::OK();
}

PlanningStatus HeuristicRRT::GetGridMap(
    const sensor_msgs::Image& map) {
    grid_map::GridMap grid_map({"elevation", "normal_x", "normal_y", "normal_z"});
    grid_map.setFrameId("")
}

}