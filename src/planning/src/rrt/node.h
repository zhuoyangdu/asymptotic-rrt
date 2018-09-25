// Copyright [2018] <Zhuoyang Du>

#ifndef SRC_PLANNING_SRC_RRT_NODE_H_
#define SRC_PLANNING_SRC_RRT_NODE_H_

#include <vector>

namespace planning {

class Node {
 public:
  Node() {}

 private:
  double x_;
  double y_;
  double theta_;
  Node *parent_node = nullptr;
  std::vector<Node *> children_;

  /// number of child nodes.
  unsigned int degree_;
};

}  // namespace planning

#endif  // SRC_PLANNING_SRC_RRT_NODE_H_
