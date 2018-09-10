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

    static void GetContours(const cv::Mat& image);

    static std::vector<Point> GetVertex(const cv::Mat& image);

    static void GetVoronoi(const cv::Mat& image);

 private:

    static void draw_point(const Mat& img, Point2f fp, Scalar color );
    static void draw_delaunay(const Mat& img, Subdiv2D& subdiv, Scalar delaunay_color );
    static void draw_voronoi(const Mat& img, Subdiv2D& subdiv );

};
}
#endif //PLANNING_IMAGE_PROC_H
