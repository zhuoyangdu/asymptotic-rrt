//
//  main.cpp
//  knn
//
//  Created by zhuoyang on 2018/10/22.
//  Copyright © 2018年 zhuoyang. All rights reserved.
//

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <functional>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class GNAT {

public:
    GNAT(const std::vector<std::pair<int, int>>& pivots, const cv::Size& size) :
            pivots_(pivots), size_(size) {
        cv::Rect rect(0, 0, size_.width, size_.height);
        cv::Subdiv2D subdiv(rect);
        for (pair<int, int> point : pivots) {
            subdiv.insert(cv::Point(point.first, point.second));
        }

        vector<Point2f> centers;
        subdiv.getVoronoiFacetList(vector<int>(), facets_, centers);
        for (int i = 0; i < centers.size(); ++i) {
            vector<pair<int, int>> range;
            for (int j = 0; j < centers.size(); ++j) {
                if (i != j) {
                    int min = INT_MAX;
                    int max = INT_MIN;
                    for (Point2f p : facets_[j]) {
                        int dist = (p.x - centers[i].x)*(p.x - centers[i].x) +
                                   (p.y - centers[i].y)*(p.y - centers[i].y);
                        if (dist > max) max = dist;
                        if (dist < min) min = dist;
                    }
                    range.push_back(pair<int, int>(min, max));
                } else {
                    range.push_back(pair<int, int>(INT_MIN, INT_MAX));
                }
            }
            distance_range_table_.push_back(range);
        }
                
        for (int i = 0; i < pivots_.size(); ++i) {
            cv::Scalar color(int((double)rand()/RAND_MAX * 255),
                             int((double)rand()/RAND_MAX * 255),
                             int((double)rand()/RAND_MAX * 255));
            colors_.push_back(color);
        }
        
        for (int i = 0; i < pivots_.size(); ++i) {
            points_.push_back({});
        }
    }
    
    void GetRange(int pivot, int region, int* min, int* max) {
        if (pivot == region) {
            std::cout << "You cant use the same pivot and region!!!!!" << std::endl;
        }
        *min = distance_range_table_[pivot][region].first;
        *max = distance_range_table_[pivot][region].second;
    }
    
    int add(const cv::Point& point) {
        int min_pivots = -1;
        int min_dist = INT_MAX;
        for (int i = 0; i < pivots_.size(); ++i) {
            pair<int, int> pivot = pivots_[i];
            int dist = (point.x - pivot.first)*(point.x - pivot.first) +
                       (point.y - pivot.second)*(point.y - pivot.second);
            if (dist < min_dist) {
                min_dist = dist;
                min_pivots = i;
            }
        }
        points_[min_pivots].push_back(point);
        point_count_++;
        return min_pivots;
    }
    
    vector<vector<Point2f>> Voronoi() {
        return facets_;
    }
    
    cv::Scalar color(int i) {
        return colors_[i];
    }
    
    vector<cv::Point> GetPointsInRegion(int i) {
        return points_[i];
    }
    
    int FindPivot(const cv::Point& point) {
        int min_pivots = -1;
        int min_dist = INT_MAX;
        for (int i = 0; i < pivots_.size(); ++i) {
            pair<int, int> pivot = pivots_[i];
            int dist = (point.x - pivot.first)*(point.x - pivot.first) +
            (point.y - pivot.second)*(point.y - pivot.second);
            if (dist < min_dist) {
                min_dist = dist;
                min_pivots = i;
            }
            
        }
        return min_pivots;
    }
    
    struct Compare {
        Compare(cv::Point sample) {this->sample = sample;}
        bool operator() (cv::Point& a, cv::Point& b) {
            int dist1 = (a.x - sample.x) * (a.x - sample.x) + (a.y - sample.y) * (a.y - sample.y);
            int dist2 = (b.x - sample.x) * (b.x - sample.x) + (b.y - sample.y) * (b.y - sample.y);
            return dist1 < dist2;
        }
        cv::Point sample;
    };
    
    int SqureDistance(const cv::Point& p1, const cv::Point& p2) const {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
    }
    
    vector<Point> kNearestPoints(const cv::Point& sample, int k) {
        int pivot_index = FindPivot(sample);
        while (point_count_ == 0) {
            return vector<Point>{};
        }
        while (points_[pivot_index].size() == 0) {
            pivot_index = int((double)rand()/RAND_MAX * pivots_.size());
        }
        Compare cmp(sample);
        std::priority_queue<cv::Point, vector<cv::Point>, decltype(cmp)> candidates(cmp);
        for (cv::Point point : points_[pivot_index]) {
            candidates.push(point);
        }
        
        while (candidates.size() > k) {
            candidates.pop();
        }
        double threshold;
        if (candidates.size() == k) {
            threshold = sqrt(SqureDistance(candidates.top(), sample));
        } else {
            threshold = INT_MAX;
        }
        
        cv::Point pivot_p(pivots_[pivot_index].first, pivots_[pivot_index].second);
        double d1 = sqrt(SqureDistance(pivot_p, sample));
        for (int i = 0; i < pivots_.size(); ++i) {
            if (i == pivot_index || points_[i].size() == 0) {
                continue;
            }
            int max, min;
            GetRange(pivot_index, i, &min, &max);
            //cout << "min:" << sqrt(min) << ", d1+t:" << d1 + threshold << endl;
            if (sqrt(min) > d1 + threshold) {
                continue;
            } else {
                for (cv::Point point : points_[i]) {
                    //cout << "p:" << sqrt(SqureDistance(point, sample)) << "," << threshold << endl;
                    if (sqrt(SqureDistance(point, sample)) < threshold) {
                        candidates.push(point);
                        if (candidates.size() > k) {
                            //cout << "pop:" << candidates.top().x << "," << candidates.top().y << endl;
                            candidates.pop();
                        }
                        //cout << "top:" << candidates.top().x << "," << candidates.top().y << endl;
                        threshold = sqrt(SqureDistance(candidates.top(), sample));
                    }
                }
            }
        }
        
        vector<cv::Point> k_nearest_points;
        while (!candidates.empty()) {
            k_nearest_points.push_back(candidates.top());
            candidates.pop();
        }
        return k_nearest_points;
    }
    
private:
    vector<pair<int, int>> pivots_;
    cv::Size size_;
    vector<vector<Point2f> > facets_;
    vector<vector<pair<int, int>>> distance_range_table_; // min, max.
    
    vector<vector<cv::Point>> points_;
    vector<cv::Scalar> colors_;
    
    int point_count_ = 0;
    
};

void PlotPoint(const cv::Mat& image,
                          const Point& point,
                          const cv::Scalar& scalar,
                          double thickness) {
    circle(image, point, thickness, scalar, CV_FILLED, CV_AA, 0);
}



void Test(const vector<int>& vx, const vector<int>& vy) {
    cv::Mat image_gray = cv::Mat::ones(512, 512, CV_8U)*255;
    cv::Mat image;
    cvtColor(image_gray, image, COLOR_GRAY2BGR);

    vector<pair<int, int>> pivots;
    for (int row : vx) {
        for (int col : vy) {
            pivots.push_back(pair<int, int>(row, col));
        }
    }
    GNAT gnat(pivots, cv::Size(image.rows, image.cols));
    /*
    vector<vector<Point2f>> facets = gnat.Voronoi();
    for (int i = 0; i < pivots.size(); ++i) {
        cv::Point p(pivots[i].first, pivots[i].second);
        PlotPoint(image, p, gnat.color(i), 5);
        for (int j = 0; j < facets[i].size()-1;++j) {
            line(image, facets[i][j], facets[i][j+1], gnat.color(i), 1);
        }
    }
     */
    auto t1 = std::chrono::system_clock::now();
    for (int i = 0; i < 5000; ++i) {
        int random_x = int((double)rand()/RAND_MAX * 511);
        int random_y = int((double)rand()/RAND_MAX * 511);
        cv::Point rand_p(random_x, random_y);
        vector<Point> k_nearest = gnat.kNearestPoints(rand_p, 5);
        int pivot_index = gnat.add(rand_p);
       // PlotPoint(image, rand_p, gnat.color(pivot_index), 1);
    }
    //PlotPoint(image, rand_p, cv::Scalar(0,0,255), 5);
    //for (Point p : k_nearest) {
    //    PlotPoint(image, p, cv::Scalar(255,0,0), 5);
    //}
    std::chrono::duration<double> elapse = std::chrono::system_clock::now() - t1;
    cout << "knn:" << elapse.count() << endl;
    
    struct Compare2 {
        Compare2(cv::Point sample) {this->sample = sample;}
        bool operator() (cv::Point& a, cv::Point& b) {
            int dist1 = (a.x - sample.x) * (a.x - sample.x) + (a.y - sample.y) * (a.y - sample.y);
            int dist2 = (b.x - sample.x) * (b.x - sample.x) + (b.y - sample.y) * (b.y - sample.y);
            return dist1 > dist2;
        }
        cv::Point sample;
    };
    
    t1 = std::chrono::system_clock::now();
    vector<cv::Point> all_points;
    for (int i = 0; i < 5000; ++i) {
        int random_x = int((double)rand()/RAND_MAX * 511);
        int random_y = int((double)rand()/RAND_MAX * 511);
        cv::Point rand_p(random_x, random_y);
        Compare2 cmp(rand_p);
        vector<cv::Point> candidates;
        std::priority_queue<cv::Point, vector<cv::Point>, decltype(cmp)> sort_points(cmp);
        vector<cv::Point> k_nearest1;
        for (cv::Point point : all_points) {
            sort_points.push(point);
        }
        all_points.push_back(rand_p);
    }
    elapse = std::chrono::system_clock::now() - t1;
    cout << "ori:" << elapse.count() << endl;
}

vector<int> linspace(int num) {
    vector<int> lins;
    int dist = 512/num/2;
    
    while (dist < 512) {
        lins.push_back(dist);
        dist += 512/num;
    }
    return lins;
}

int main(int argc, const char * argv[]) {
    srand(time(0));
    vector<int> vx = {85, 256, 426};
    vector<int> vy = {85, 256, 426};
    
    
    for (int i = 3; i <= 6; ++i) {
        cout << "num:" << i << endl;
        vector<int> split = linspace(i);
        Test(split, split);
    }
    
    cv::waitKey();
    return 0;
}
