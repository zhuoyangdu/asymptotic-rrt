// Copyright [2018] <Zhuoyang Du>

#ifndef SRC_PLANNING_SRC_RRT_PROBABLISTIC_MAP_H_
#define SRC_PLANNING_SRC_RRT_PROBABLISTIC_MAP_H_

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "node.h"

namespace planning {

class ProbablisticMap {
 public:
    explicit ProbablisticMap(const cv::Mat& attractive_prob);

    Node Sampling();

 private:
    cv::Mat attractive_prob_;
    int prob_sum_;
    std::vector<int> prob_cumsum_;

    void InitMap();

    int FindRandSection(int num, int lower, int upper);

};

}  // namespace planning


#endif  // SRC_PLANNING_SRC_RRT_PROBABLISTIC_MAP_H_
