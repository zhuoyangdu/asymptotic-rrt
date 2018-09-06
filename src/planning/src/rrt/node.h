//
// Created by zy on 18-9-6.
//

#ifndef PLANNING_RRT_NODE_H_
#define PLANNING_RRT_NODE_H_

#include <vector>

namespace planning {

class Node {
 public:
    Node() {};

 private:
    double x_;
    double y_;
    double theta_;
    Node* parent_node = nullptr;
    std::vector<Node *> children_;

    /// number of child nodes.
    unsigned int degree_;

};

}

#endif //PLANNING_RRT_NODE_H_
