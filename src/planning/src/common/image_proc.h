//
// Created by zy on 18-9-10.
//

#ifndef PLANNING_IMAGE_PROC_H
#define PLANNING_IMAGE_PROC_H

#include <iostream>
#include <string>
#include <cstdlib>
#include <vector>
#include <set>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

namespace planning {
class ImageProc {
 public:
    ImageProc() = delete;

    static void FromROSImageToOpenCV(const sensor_msgs::Image &image,
                                     cv_bridge::CvImagePtr cv_image,
                                     const std::string& opencv_win);

    static std::vector<Point> GetVertex(const cv::Mat& image);

    static cv::Mat_<double> GetVoronoiProbMap(const cv::Mat& image);

    static cv::Mat_<double> GetTargetAttractiveMap(
        const cv::Mat& image, const cv::Point& goal);

    static cv::Mat_<double> GetAttractiveProbMap(
        const cv::Mat& image, const cv::Point& goal,
        double k_voronoi, double k_goal);

 private:


};
}
#endif //PLANNING_IMAGE_PROC_H
