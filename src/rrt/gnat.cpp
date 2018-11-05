#include "gnat.h"

using namespace std;

namespace planning {

vector<Node> GNAT::kNearestPoints(const Node& sample, int k) {
    int pivot_index = FindPivot(sample);
    if (point_count_ == 0) {
        return vector<Node>{};
    }
    if (nodes_[pivot_index].size() == 0) {
        pivot_index = init_pivots_;
    }
    Compare cmp(sample);
    std::priority_queue<Node, vector<Node>, decltype(cmp)> candidates(cmp);
    for (Node point : nodes_[pivot_index]) {
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
        if (i == pivot_index || nodes_[i].size() == 0) {
            continue;
        }
        int max, min;
        GetRange(pivot_index, i, &min, &max);
        if (sqrt(min) > d1 + threshold) {
            continue;
        } else {
            for (Node node : nodes_[i]) {
                if (sqrt(SqureDistance(node, sample)) < threshold) {
                    candidates.push(node);
                    if (candidates.size() > k) {
                        candidates.pop();
                    }
                    threshold = sqrt(SqureDistance(candidates.top(), sample));
                }
            }
        }
    }
    vector<Node> k_nearest_points;
    while (!candidates.empty()) {
        k_nearest_points.push_back(candidates.top());
        candidates.pop();
    }
    return k_nearest_points;
}

Node GNAT::NearestPoint(const Node& sample) {
    int pivot_index = FindPivot(sample);
    while (nodes_[pivot_index].size() == 0) {
        pivot_index = init_pivots_;
    }

    min_dis_ = INT_MAX;
    //double threshold = INT_MAX;
    NearestInPivot(sample, pivot_index);

    cv::Point pivot_p(pivots_[pivot_index].first, pivots_[pivot_index].second);
    double d1 = sqrt(SqureDistance(pivot_p, sample));
    double threshold = d1 + min_dis_;

    int thread_nums = 5;

    int p = 0;
    while (p < pivots_.size()) {
        int n = 0;
        std::thread threads[thread_nums];
        while (n < thread_nums && p < pivots_.size()) {
            if (PivotIsFeasible(p, pivot_index, d1+min_dis_)) {
                threads[n] = std::thread(&planning::GNAT::NearestInPivot, this,
                                         sample, p);
                n++;
                p++;
            } else {
                p++;
            }
        }
        for (std::thread& t : threads) {
            if (t.joinable())
                t.join();
        }
    }
    return min_node_;
}

void GNAT::NearestInPivot(const Node& sample,
                          int pivot_index) {
    int min_dist = INT_MAX;
    Node min_node;
    for (Node node : nodes_[pivot_index]) {
        int dist = sqrt(SqureDistance(node, sample));
        if (dist < min_dist) {
            min_node = node;
            min_dist = dist;
        }
    }
    mutex_.lock();
    if (min_dist < min_dis_) {
        min_dis_ = min_dist;
        min_node_ = min_node;
    }
    mutex_.unlock();
}

std::vector<std::vector<int>> GNAT::DividePivots(
        const std::vector<int>& pivots,
        int nums) {
    std::vector<std::vector<int>> divided_pivots;
    std::vector<int> s;
    for (int i = 0; i < pivots.size(); ++i) {
        s.push_back(pivots[i]);
        if (s.size() == nums) {
            divided_pivots.push_back(s);
            s.clear();
        }
    }
    if (s.size() != 0) {
        divided_pivots.push_back(s);
    }
    return divided_pivots;
}

bool GNAT::PivotIsFeasible(int candidate_pivot,
                           int sample_pivot,
                           double threshold) {
    if (sample_pivot == candidate_pivot ||
        nodes_[candidate_pivot].size() == 0) {
        return false;
    }
    int max, min;
    GetRange(sample_pivot, candidate_pivot, &min, &max);
    if (sqrt(min) > threshold) {
        return false;
    } else {
        return true;
    }
}



}




