// Copyright [2018] <Zhuoyang Du>

#ifndef SRC_PLANNING_SRC_COMMON_IMAGE_PROC_H_
#define SRC_PLANNING_SRC_COMMON_IMAGE_PROC_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <grid_map_msgs/GridMap.h>

#include <iostream>
#include <string>
#include <cstdlib>
#include <vector>
#include <set>
#include <cmath>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

namespace planning {
class ImageProc {
 public:
    ImageProc() = delete;

    static cv::Mat FromROSImageToOpenCV(const sensor_msgs::Image &image);

    static std::vector<Point> GetVertex(const cv::Mat& image);

    static cv::Mat_<double> GetVoronoiProbMap(const cv::Mat& image);

    static cv::Mat_<double> GetTargetAttractiveMap(
        const cv::Mat& image, const cv::Point& goal);

    static cv::Mat_<double> GetAttractiveProbMap(
        const cv::Mat& image, const cv::Point& goal,
        double k_voronoi, double k_goal);

    static void PlotPoint(const cv::Mat& image,
                          const Point& point,
                          const cv::Scalar& scalar);

    static grid_map_msgs::GridMap ImageToGridMapMsg(const cv::Mat& image);

 private:
};
}  // namespace planning
#endif  // SRC_PLANNING_SRC_COMMON_IMAGE_PROC_H_
