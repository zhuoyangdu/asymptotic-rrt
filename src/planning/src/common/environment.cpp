// Copyright [2018] <Zhuoyang Du>

#include "environment.h"

namespace planning {

Environment::Environment(const sensor_msgs::Image& image,
                         const PlanningConf& planning_conf)
                         : planning_conf_(planning_conf) {
    resolutionX_   = planning_conf.vrep_conf().resolutionx();
    resolutionY_   = planning_conf.vrep_conf().resolutiony();
    rangeX_.first  = planning_conf.vrep_conf().minx();
    rangeX_.second = planning_conf.vrep_conf().maxx();
    rangeY_.first  = planning_conf.vrep_conf().miny();
    rangeY_.second = planning_conf.vrep_conf().maxy();

    goal_.x = planning_conf.goal().x();
    goal_.y = planning_conf.goal().y();
    GetPixelCoord(goal_.x, goal_.y, &pixel_goal_.x, &pixel_goal_.y);

    is_init_ = true;
    map_static_ = ImageProc::FromROSImageToOpenCV(image);

    GenerateAttractiveProbMap();
}

void Environment::UpdateDynamicMap(const sensor_msgs::Image& image) {
    map_dynamic_ = ImageProc::FromROSImageToOpenCV(image);
}

void Environment::GetPixelCoord(double x, double y,
                                double *row, double *col) {
    *row = (rangeY_.second - y)
         / (rangeY_.second - rangeY_.first) * resolutionY_;
    *col = (x - rangeX_.first) * resolutionX_
         / (rangeX_.second - rangeX_.first);
}

void Environment::GetWorldCoord(double row, double col,
                                double *x, double *y) {
    *y = rangeY_.second - row / resolutionY_ * (rangeY_.second - rangeY_.first);
    *x = rangeX_.first + col / resolutionX_ * (rangeX_.second - rangeX_.first);
}

void Environment::GenerateAttractiveProbMap() {
    attractive_map_ = ImageProc::GetAttractiveProbMap(
        map_static_, pixel_goal_,
        planning_conf_.rrt_conf().k_voronoi(),
        planning_conf_.rrt_conf().k_goal());
}

bool Environment::CheckCollisionByPixelCoord(double row, double col) {
    int value = static_cast<int>(map_dynamic_.at<uchar>(static_cast<int>(row),
                                                        static_cast<int>(col)));
    if (value == 0) {
        // Is collided.
        return true;
    } else {
        return false;
    }
}

bool Environment::CheckCollisionByPixelCoord(const cv::Point& point) {
    return CheckCollisionByPixelCoord(point.x, point.y);
}

bool Environment::CheckCollisionByWorldCoord(double x, double y) {
    double row, col;
    GetPixelCoord(x, y, &row, &col);
    return CheckCollisionByPixelCoord(static_cast<int>(row),
                                      static_cast<int>(col));
}

}  // namespace planning
