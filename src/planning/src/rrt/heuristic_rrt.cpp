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
    Environment* environment) {

    cv::Point2d init;
    environment->GetPixelCoord(vehicle_state.x, vehicle_state.y, &init.x, &init.y);

    cv::Mat img_env = environment->DynamicMap();
    ImageProc::PlotPoint(img_env, init, Scalar(255));
    imshow("environment", img_env);

    return PlanningStatus::OK();
}

PlanningStatus HeuristicRRT::GetGridMap(
    const sensor_msgs::Image& image) {

    return PlanningStatus::OK();
}

}