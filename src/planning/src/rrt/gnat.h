//  Created by zhuoyang on 2018/10/22.
//  Copyright © 2018年 zhuoyang. All rights reserved.
//

#ifndef SRC_PLANNING_SRC_RRT_GNAT_H_
#define SRC_PLANNING_SRC_RRT_GNAT_H_

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <functional>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "node.h"

using namespace std;
using namespace cv;

namespace planning {

class GNAT {

public:
    GNAT(int pivots_k, const cv::Size& size) :
        size_(size) {
        
        vector<int> px = linspace(pivots_k);
        vector<int> py = px;
        pivots_.clear();
        for (int i = 0; i < px.size(); ++i) {
            for (int j = 0; j < py.size(); ++j) {
                pivots_.push_back(pair<int,int>(px[i], py[j]));
            }
        }

        cv::Rect rect(0, 0, size_.width, size_.height);
        cv::Subdiv2D subdiv(rect);
        for (pair<int, int> point : pivots_) {
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
    
    vector<int> linspace(int num) {
        vector<int> lins;
        int dist = 512/num/2;
    
        while (dist < 512) {
            lins.push_back(dist);
            dist += 512/num;
        }
        return lins;
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
            int dist1 = (a.x - sample.x) * (a.x - sample.x) + 
                        (a.y - sample.y) * (a.y - sample.y);
            int dist2 = (b.x - sample.x) * (b.x - sample.x) + 
                        (b.y - sample.y) * (b.y - sample.y);
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
            if (sqrt(min) > d1 + threshold) {
                continue;
            } else {
                for (cv::Point point : points_[i]) {
                    if (sqrt(SqureDistance(point, sample)) < threshold) {
                        candidates.push(point);
                        if (candidates.size() > k) {
                            candidates.pop();
                        }
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




} // namespace

#endif // SRC_PLANNING_SRC_RRT_GNAT_H_
