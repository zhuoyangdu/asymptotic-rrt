// Copyright [2018] <Zhuoyang Du>

#ifndef SRC_PLANNING_SRC_RRT_NODE_H_
#define SRC_PLANNING_SRC_RRT_NODE_H_

#include <vector>

namespace planning {

class Node {
 public:
  Node() {}

  Node(int row, int col) {
      row_ = row;
      col_ = col;
  }

  void set_theta(double theta) { theta_ = theta; }

  int row() { return row_; }

  int col() { return col_; }

  int theta() { return theta_; }
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
