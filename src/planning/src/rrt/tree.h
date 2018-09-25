// Copyright [2018] <Zhuoyang Du>

#ifndef SRC_PLANNING_SRC_RRT_TREE_H_
#define SRC_PLANNING_SRC_RRT_TREE_H_

#include "node.h"

namespace planning {

class Tree {
 public:
    explicit Tree(const Node& root);


 private:
    Node root_;
};

}  // namespace planning


#endif  // SRC_PLANNING_SRC_RRT_TREE_H_
