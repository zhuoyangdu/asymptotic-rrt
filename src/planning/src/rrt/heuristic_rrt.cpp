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

    // Get init state.
    cv::Point2d init;
    environment->GetPixelCoord(vehicle_state.x, vehicle_state.y,
                               &init.x, &init.y);

    cv::Mat img_env = environment->DynamicMap();
    // ImageProc::PlotPoint(img_env, init, Scalar(255));
    // imshow("environment", img_env);

    // Get sampling probablistic map.
    cv::Mat goal_prob       = environment->TargetAttractiveMap();
    cv::Mat voronoi_prob    = environment->VoronoiAttractiveMap();
    cv::Mat attractive_prob = environment->AttractiveMap();
    grid_map_msgs::GridMap attractive_msg = ImageProc::ImageToGridMapMsg(
                                            attractive_prob);

    imshow("attractive", attractive_prob);
    pub_map_.publish(attractive_msg);

    return PlanningStatus::OK();
}

PlanningStatus HeuristicRRT::GetGridMap(
    const sensor_msgs::Image& image) {

    return PlanningStatus::OK();
}

}  // namespace planning
