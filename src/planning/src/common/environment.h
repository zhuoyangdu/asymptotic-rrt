// Copyright [2018] <Zhuoyang Du>

#ifndef SRC_PLANNING_SRC_COMMON_ENVIRONMENT_H_
#define SRC_PLANNING_SRC_COMMON_ENVIRONMENT_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <planning_conf.pb.h>

#include <iostream>
#include <vector>
#include <utility>

#include "image_proc.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace planning {
class Environment {
 public:
    // The default position of the map center is (0,0,0).
    Environment() = default;

    Environment(const sensor_msgs::Image& image,
                const PlanningConf& planning_conf);

    ~Environment() = default;

    void GetPixelCoord(double x, double y,
                       double* row, double* col);

    void GetWorldCoord(double row, double col,
                       double* x, double* y);

    bool CheckCollisionByPixelCoord(double row, double col);

    bool CheckCollisionByPixelCoord(const cv::Point& point);

    bool CheckCollisionByWorldCoord(double x, double y);

    void UpdateDynamicMap(const sensor_msgs::Image& image);

    cv::Mat DynamicMap() { return map_dynamic_; }

    cv::Mat_<double> AttractiveMap() {return attractive_map_; }

 private:
    void GenerateAttractiveProbMap();

    // Params
    PlanningConf planning_conf_;
    int resolutionX_ = 512;
    int resolutionY_ = 512;
    std::pair<double, double> rangeX_;
    std::pair<double, double> rangeY_;

    bool is_init_ = false;
    cv::Mat map_static_;
    cv::Mat map_dynamic_;
    cv::Point2d goal_;
    cv::Point2d pixel_goal_;

    cv::Mat_<double> attractive_map_;
};

}  // namespace planning

#endif  // SRC_PLANNING_SRC_COMMON_ENVIRONMENT_H_
