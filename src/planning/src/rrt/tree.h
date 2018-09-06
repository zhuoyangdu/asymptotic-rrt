//
// Created by zy on 18-9-6.
//

#ifndef PLANNING_RRT_TREE_H_
#define PLANNING_RRT_TREE_H_

#include "node.h"

namespace planning {

class Tree {
 public:
    explicit Tree(const Node& root);


 private:
    Node root_;
};

}


#endif //PLANNING_RRT_TREE_H_
