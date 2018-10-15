// Copyright [2018] <Zhuoyang Du>

#ifndef SRC_PLANNING_SRC_RRT_NODE_H_
#define SRC_PLANNING_SRC_RRT_NODE_H_

#include <vector>
#include <cmath>
#include <iostream>

namespace planning {

class Node {
 public:
  Node() {}

  Node(int row, int col, double theta) {
      row_ = row;
      col_ = col;
      theta_ = theta;
  }

  Node(int row, int col) {
      row_ = row;
      col_ = col;
  }

  void SetTheta(double theta) { theta_ = theta; }

  void SetRow(double row) { row_ = row; }

  void SetCol(double col) { col_ = col; }

  int row() const { return row_; }

  int col() const { return col_; }

  int theta() const { return theta_; }

  static double SquareDistance(const Node& a, const Node& b) {
      return (a.col() - b.col()) * (a.col() - b.col())
            + (a.row() - b.row()) * (a.row() - b.row());
  }

  // 沿x轴， 顺时针为正
  static double GetDeltaTheta(const Node& parent, const Node& child) {
      double dtheta = atan2(child.row() - parent.row(), child.col() - parent.col());
      double theta = fabs(parent.theta() - dtheta);
      if (theta > M_PI) theta -= M_PI;
      return theta;
  }

  static double ComputeTheta(const Node& parent, const Node& child) {
      return atan2(child.col() - child.col(), parent.row() - parent.row());
  }

 private:
  int row_;
  int col_;
  double theta_;
  Node *parent_node = nullptr;
  std::vector<Node *> children_;

  /// number of child nodes.
  unsigned int degree_;
};

}  // namespace planning

#endif  // SRC_PLANNING_SRC_RRT_NODE_H_
