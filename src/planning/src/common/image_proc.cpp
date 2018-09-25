// Copyright [2018] <Zhuoyang Du>

#include "image_proc.h"

using namespace std;
using namespace cv;

namespace planning {

void ImageProc::PlotPoint(const cv::Mat& image,
                          const Point& point,
                          const cv::Scalar& scalar) {
    circle(image, point, 3, scalar, CV_FILLED, CV_AA, 0);
}

cv::Mat ImageProc::FromROSImageToOpenCV(const sensor_msgs::Image &image) {
    cv_bridge::CvImagePtr cv_image;
    try {
        cv_image = cv_bridge::toCvCopy(
            image, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    return cv_image->image;
}

std::vector<Point> ImageProc::GetVertex(const cv::Mat &image) {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy,
                     RETR_TREE, CV_CHAIN_APPROX_NONE);

    vector<Point> vertex;
    for (vector<Point> contour : contours) {
        for (int i = 0; i < contour.size(); i = i+2) {
            vertex.push_back(contour[i]);
        }
    }
    return vertex;
}

cv::Mat_<double> ImageProc::GetVoronoiProbMap(const cv::Mat& image) {
    // Get the vertex point of the image.
    cv::Size size = image.size();
    cv::Rect rect(0, 0, size.width, size.height);
    cv::Subdiv2D subdiv(rect);
    std::vector<cv::Point> vertex = GetVertex(image);
    for (cv::Point point : vertex) {
        subdiv.insert(point);
    }

    // Get Voronoi map of image.
    vector<vector<Point2f> > facets;
    vector<Point2f> centers;
    subdiv.getVoronoiFacetList(vector<int>(), facets, centers);
    Mat img_vor(image.rows, image.cols, CV_8U, Scalar(255));
    vector<Point2f> circle_center;
    for (vector<Point2f> p1 : facets) {
        for (Point2f p2 : p1) {
            if (p2.x > 511 || p2.x < 0 || p2.y > 511 || p2.y < 0) {
                continue;
            }
            int value = static_cast<int>(image.at<uchar>(p2));
            if (value > 0) {
                circle(img_vor, p2, 3, Scalar(0), CV_FILLED, CV_AA, 0);
            } else {
                continue;
            }
            circle_center.push_back(p2);
        }
    }

    int kern_dim = 50;
    cv::Mat_<double> kern(kern_dim*2+1, kern_dim*2+1);
    for (int i = 0; i < kern_dim*2+1; ++i) {
        for (int j = 0; j < kern_dim*2+1; ++j) {
            int den = (i - kern_dim) * (i - kern_dim)
                    + (j - kern_dim) * (j - kern_dim);
            double k = (den == 0) ? 1.0 : 1.0 / sqrt(static_cast<double>(den));
            kern.at<double>(i, j) = k / 255;
        }
    }

    Mat filter_image(image.rows, image.cols, CV_64F);
    filter2D(img_vor , filter_image, img_vor.depth(), kern);


    cv::Mat_<double> voronoi_prob(image.rows, image.cols);
    double min, max;
    cv::minMaxIdx(filter_image, &min, &max);
    voronoi_prob = 0.8 * (filter_image - min) / max + 0.2;
    cv::minMaxIdx(voronoi_prob, &min, &max);
    std::cout << "GetVoronoiProbMap : min:" << min
        << ", max:" << max << std::endl;

    imshow("voronoi_prob", filter_image*100);
    imshow("voronoi", voronoi_prob*100);

    return voronoi_prob;
}

cv::Mat_<double> ImageProc::GetTargetAttractiveMap(
    const cv::Mat& image, const cv::Point& goal) {
    cv::Mat_<double> goal_prob(image.rows, image.cols);
    for (int i = 0; i < goal_prob.rows; ++i) {
        for (int j = 0; j < goal_prob.cols; ++j) {
            goal_prob.at<double>(i, j) =
                1.0 / (sqrt(0.5 * ((goal.x - i) * (goal.x - i)
                          + (goal.y - j) * (goal.y - j)) + 1));
        }
    }
    double min, max;
    cv::minMaxIdx(goal_prob, &min, &max);
    std::cout << "GetTargetAttractiveMap: min:"
        << min << ", max:" << max << std::endl;
    imshow("goal", 100 * goal_prob);
    return  goal_prob;
}

cv::Mat_<double> ImageProc::GetAttractiveProbMap(
    const cv::Mat& image, const cv::Point& goal,
    double k_voronoi, double k_goal) {
    cv::Mat_<double> voronoi_prob_map
        = planning::ImageProc::GetVoronoiProbMap(image);
    cv::Mat_<double> goal_prob_map
        = planning::ImageProc::GetTargetAttractiveMap(image, goal);
    cv::Mat_<double> attractive_prob(image.rows, image.cols);
    attractive_prob = k_voronoi * voronoi_prob_map + k_goal * goal_prob_map;
    double min, max;
    cv::minMaxIdx(attractive_prob, &min, &max);
    std::cout << "GetAttractiveProbMap: min:"
        << min << ", max:" << max << std::endl;

    // Remove collision.
    for (int i = 0; i < attractive_prob.rows; ++i) {
        for (int j = 0; j < attractive_prob.cols; ++j) {
            if (static_cast<int>(image.at<uchar>(i, j)) <= 0) {
                attractive_prob(i, j) = 0;
            }
        }
    }
    attractive_prob = attractive_prob * (1.0 / max);
    imshow("prob map", attractive_prob);
}

grid_map_msgs::GridMap ImageProc::ImageToGridMapMsg(const cv::Mat& image) {
    grid_map::GridMap map({"evaluation"});
    map.setFrameId("world");
    map.setGeometry(grid_map::Length(20, 20), 1.0 / 512);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             map.getLength().x(), map.getLength().y(),
             map.getSize()(0), map.getSize()(1));
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(
                                                    image, "layer", map);
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map, message);
    return  message;
}

}  // namespace planning
