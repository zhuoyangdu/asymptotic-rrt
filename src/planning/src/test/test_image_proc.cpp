//
// Created by zy on 18-9-10.
//

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <signal.h>

#include "../common/image_proc.h"

/*
void sig_handler(int sig) {
    if (sig == SIGINT) {
        cv::destroyAllWindows();
    }
}
*/

int main(int argv, char** argc) {
    // signal(SIGINT, sig_handler);

    std::string pkg_path = ros::package::getPath("planning");
    std::string map_path = pkg_path + "/resources/map/map.bmp";

    const std::string win1 = "origin_map";
    cv::namedWindow(win1);

    // gray image.
    cv::Mat cv_image = cv::imread(map_path, 0);
    cv::imshow(win1, cv_image);

    planning::ImageProc::GetVoronoi(cv_image);

    cv::waitKey(0);
    cv::destroyWindow(win1);
    return 0;
}