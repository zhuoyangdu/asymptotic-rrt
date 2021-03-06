// Copyright [2018] <Zhuoyang Du>

#ifndef SRC_PLANNING_SRC_COMMON_IMAGE_PROC_H_
#define SRC_PLANNING_SRC_COMMON_IMAGE_PROC_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
// #include <grid_map_msgs/GridMap.h>

#include <iostream>
#include <string>
#include <cstdlib>
#include <vector>
#include <set>
#include <cmath>

// #include <grid_map_ros/grid_map_ros.hpp>
// #include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../rrt/node.h"

using namespace cv;

namespace planning {
class ImageProc {
 public:
    ImageProc() = delete;

    static cv::Mat FromROSImageToOpenCV(const sensor_msgs::Image &image);

    static std::vector<Point> GetVertex(const cv::Mat& image);

    static cv::Mat GetVoronoiProbMap(const cv::Mat& image);

    static cv::Mat GetTargetAttractiveMap(
        const cv::Mat& image, const cv::Point& goal);

    static cv::Mat GetAttractiveProbMap(
        const cv::Mat& image, const cv::Point& goal,
        double k_voronoi, double k_goal);

    static void GetAttractiveProbMap(
            const cv::Mat& image, const cv::Point& goal,
            double k_voronoi, double k_goal,
            cv::Mat* goal_prob_map,
            cv::Mat* voronoi_prob_map,
            cv::Mat* attractive_prob_map);

    static void GetObstacleRepulsiveField(const cv::Mat& image,
                                          cv::Mat* repulsive_filed_x,
                                          cv::Mat* repulsive_filed_y);

    static void PlotPoint(const cv::Mat& image,
                          const Point& point,
                          const cv::Scalar& scalar,
                          double thickness);

    static void PlotPoint(const cv::Mat& image,
                          const Node& node,
                          const cv::Scalar& scalar,
                          double thickness);

    static void PlotLine(const cv::Mat& image,
                         const Node& a,
                         const Node& b,
                         const cv::Scalar& scalar,
                         double thickness);

    static void PlotPath(const cv::Mat& image,
                         const std::vector<Node> path,
                         const cv::Scalar& scalar,
                         double thickness);

    static void PlotPath(const cv::Mat& image,
                         const std::vector<double> x,
                         const std::vector<double> y,
                         const cv::Scalar& scalar,
                         double thickness);

    // static grid_map_msgs::GridMap ImageToGridMapMsg(const cv::Mat& image);
};
}  // namespace planning
#endif  // SRC_PLANNING_SRC_COMMON_IMAGE_PROC_H_
