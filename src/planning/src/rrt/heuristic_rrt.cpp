// Copyright [2018] <Zhuoyang Du>

#include "heuristic_rrt.h"

namespace planning {

HeuristicRRT::HeuristicRRT(const RRTConf& rrt_conf)
        : rrt_conf_(rrt_conf), is_init_(true) {
    pub_map_ = private_nh_.advertise<grid_map_msgs::GridMap>(
                "/planning/map", 1, true);
}

PlanningStatus HeuristicRRT::Solve(
    const geometry_msgs::Pose2D& vehicle_state,
    Environment* environment) {

    cv::Point2d init;
    environment->GetPixelCoord(vehicle_state.x, vehicle_state.y,
                               &init.x, &init.y);

    cv::Mat img_env = environment->DynamicMap();
    // ImageProc::PlotPoint(img_env, init, Scalar(255));
    // imshow("environment", img_env);

    grid_map_msgs::GridMap message = ImageProc::ImageToGridMapMsg(img_env);

    cv::Mat attractive_prob = environment->TargetAttractiveMap();
    grid_map_msgs::GridMap goal_prob = ImageProc::ImageToGridMapMsg(
                                      attractive_prob);
    imshow("goal", attractive_prob);
    pub_map_.publish(goal_prob);

    return PlanningStatus::OK();
}

PlanningStatus HeuristicRRT::GetGridMap(
    const sensor_msgs::Image& image) {

    return PlanningStatus::OK();
}

}  // namespace planning
