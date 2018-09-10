//
// Created by zy on 18-9-10.
//

#ifndef PLANNING_ENVIRONMENT_H
#define PLANNING_ENVIRONMENT_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
namespace planning {

class Environment {
 public:
    // The default position of the map center is (0,0,0).
    Environment(const sensor_msgs::Image &image,
                int resolution) {
        cv::namedWindow(OPENCV_WINDOW);
        cv_bridge::CvImagePtr cv_image;
        FromROSImageToOpenCV(image, cv_image);
    }

    ~Environment() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void FromROSImageToOpenCV(
        const sensor_msgs::Image &image,
        cv_bridge::CvImagePtr cv_image
    ) {
        try {
            cv_image = cv_bridge::toCvCopy(
                image, sensor_msgs::image_encodings::MONO16);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::imshow(OPENCV_WINDOW, cv_image->image);
        cv::waitKey(3);
    }

 private:
    int resolution_ = 512;
    sensor_msgs::Image image_;
    int height_;
    int width_;

};

}

#endif //PLANNING_ENVIRONMENT_2D_H
