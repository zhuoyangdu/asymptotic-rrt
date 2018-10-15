// Copyright [2018] <Zhuoyang Du>

#include <ros/ros.h>
#include <csignal>
#include <unistd.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "planning_node.h"

void sighandler(int signum) {
    std::cout << "press " << signum << std::endl;
    ros::shutdown();
    cv::destroyAllWindows();
    exit(signum);
}

int main(int argc, char** argv) {
    signal(SIGINT, sighandler);

    ros::init(argc, argv, "planning_node");
    ros::NodeHandle nh;
    planning::PlanningNode planning_node(nh);
    planning_node.Run();
    return 0;
}
