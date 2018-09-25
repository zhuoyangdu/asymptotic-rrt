// Copyright [2018] <Zhuoyang Du>

#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>

#include <iostream>
#include <string>
#include <chrono>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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

    // gray image.
    cv::Mat cv_image = cv::imread(map_path, CV_8U);
    // cv::imshow("origin_map", cv_image);

    auto start = std::chrono::system_clock::now();
    cv::Mat_<double> prob_map = planning::ImageProc::GetAttractiveProbMap(
        cv_image, Point(100, 400), 1.0, 5.0);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::cout << "elapsed seconds:" << elapsed_seconds.count() << "s\n";

    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}
