//
// Created by zy on 18-9-10.
//

#include "image_proc.h"

using namespace std;
using namespace cv;

namespace planning {
// Draw a single point
void ImageProc::draw_point(const Mat& img, Point2f fp, Scalar color )
{
    circle( img, fp, 2, color, CV_FILLED, CV_AA, 0 );
}

// Draw delaunay triangles
void ImageProc::draw_delaunay(const Mat& img, Subdiv2D& subdiv, Scalar delaunay_color )
{

    vector<Vec6f> triangleList;
    subdiv.getTriangleList(triangleList);
    vector<Point> pt(3);
    Size size = img.size();
    Rect rect(0,0, size.width, size.height);

    for( size_t i = 0; i < triangleList.size(); i++ )
    {
        Vec6f t = triangleList[i];
        pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
        pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
        pt[2] = Point(cvRound(t[4]), cvRound(t[5]));

        // Draw rectangles completely inside the image.
        if ( rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2]))
        {
            line(img, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
            line(img, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
            line(img, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
        }
    }
}

//Draw voronoi diagram
void ImageProc::draw_voronoi(const Mat& img, Subdiv2D& subdiv )
{
    vector<vector<Point2f> > facets;
    vector<Point2f> centers;
    subdiv.getVoronoiFacetList(vector<int>(), facets, centers);

    vector<Point> ifacet;
    vector<vector<Point> > ifacets(1);

    for( size_t i = 0; i < facets.size(); i++ )
    {
        ifacet.resize(facets[i].size());
        for( size_t j = 0; j < facets[i].size(); j++ )
            ifacet[j] = facets[i][j];

        Scalar color;
        color[0] = rand() & 255;
        color[1] = rand() & 255;
        color[2] = rand() & 255;
        fillConvexPoly(img, ifacet, color, 8, 0);

        ifacets[0] = ifacet;
        polylines(img, ifacets, true, Scalar(), 1, CV_AA, 0);
        circle(img, centers[i], 3, Scalar(), CV_FILLED, CV_AA, 0);
    }
}

std::vector<Point> ImageProc::GetVertex(const cv::Mat &image) {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy,  RETR_TREE, CHAIN_APPROX_SIMPLE);
    Mat img_con=Mat::ones(image.size(),CV_8UC1)*255;
    cv::drawContours(img_con, contours, -1, Scalar(0,255,0),3);
    imshow("contours", img_con);

    vector<Point> vertex;
    for (vector<Point> contour : contours) {
        for (Point p : contour) {
            vertex.push_back(p);
        }
    }
    return vertex;
}

void ImageProc::FromROSImageToOpenCV(const sensor_msgs::Image &image,
                                     cv_bridge::CvImagePtr cv_image,
                                     const std::string &opencv_win) {
    try {
        cv_image = cv_bridge::toCvCopy(
            image, sensor_msgs::image_encodings::MONO16);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::imshow(opencv_win, cv_image->image);
    cv::waitKey(3);
}

void ImageProc::GetVoronoi(const cv::Mat& image) {
    string win_delaunay = "Delaunay Triangulation";
    string win_voronoi = "Voronoi Diagram";

    cv::Size size = image.size();
    cv::Rect rect(0, 0, size.width, size.height);
    cv::Subdiv2D subdiv(rect);
    std::vector<cv::Point> vertex = GetVertex(image);
    std::cout << "vertex size:" << vertex.size() << std::endl;
    for (cv::Point point : vertex) {
        subdiv.insert(point);
    }

    // Draw delaunay triangles
    Mat img_del = Mat::zeros(image.rows, image.cols, CV_8UC3);
    draw_delaunay( img_del, subdiv, Scalar(255,255,255) );

    // Draw points
    for( vector<Point>::iterator it = vertex.begin(); it != vertex.end(); it++)
    {
        draw_point(img_del, *it, Scalar(0,0,255));
    }

    // Allocate space for Voronoi Diagram
    Mat img_voronoi = Mat::zeros(image.rows, image.cols, CV_8UC3);

    // Draw Voronoi diagram
    draw_voronoi( img_voronoi, subdiv );

    // Show results.
    imshow( win_delaunay, img_del);
    imshow( win_voronoi, img_voronoi);
    waitKey(0);

}

}