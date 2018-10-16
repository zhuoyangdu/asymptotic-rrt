// Copyright [2018] <Zhuoyang Du>

#include "heuristic_rrt.h"
#include <queue>
#include <functional>

using namespace std;

namespace planning {

HeuristicRRT::HeuristicRRT(const PlanningConf& planning_conf)
        : rrt_conf_(planning_conf.rrt_conf()), is_init_(true),
          planning_conf_(planning_conf) {
    pub_map_ = private_nh_.advertise<grid_map_msgs::GridMap>(
                "/planning/map", 1, true);
    show_image_ = rrt_conf_.show_image();
}

PlanningStatus HeuristicRRT::Solve(
    const geometry_msgs::Pose2D& vehicle_state,
    Environment* environment) {

    // Get init state.
    cv::Point2d init;
    environment->GetPixelCoord(vehicle_state.x, vehicle_state.y,
                               &init.y, &init.x);

    cv::Mat img_env;
    cvtColor(environment->DynamicMap(), img_env, COLOR_GRAY2BGR);
    ImageProc::PlotPoint(img_env, init, Scalar(0, 100, 0), 2);

    // Get sampling probablistic map.
    cv::Mat goal_prob       = environment->TargetAttractiveMap();
    cv::Mat voronoi_prob    = environment->VoronoiAttractiveMap();
    cv::Mat attractive_prob = environment->AttractiveMap();

    grid_map_msgs::GridMap attractive_msg = ImageProc::ImageToGridMapMsg(
                                            attractive_prob);
    if (show_image_)
        imshow("attractive", attractive_prob);
    pub_map_.publish(attractive_msg);
    ProbablisticMap probablistic_map(attractive_prob);

    Node init_node(int(init.x), int(init.y), vehicle_state.theta);
    init_node.SetIndex(0);
    init_node.SetParent(-1);

    std::vector<Node> tree = {init_node};

    Node goal_node = environment->Goal();
    std::cout << "init:" << vehicle_state.x << "," << vehicle_state.y << std::endl;
    std::cout << "init pixel:" << init_node.row() << ", " << init_node.col() << std::endl;
    std::cout << "goal pixel:" << goal_node.row() << ", " << goal_node.col() << std::endl;

    int i = 0;
    while (i < rrt_conf_.max_attemp()) {
        if ( i % 200 == 0) {
            std::cout << "iteration " << i << " times." << std::endl;
            i ++;
        }

        // Heuristic sample.
        Node sample = probablistic_map.Sampling();
        // ImageProc::PlotPoint(img_env, cv::Point(sample.col(), sample.row()),
        //                     Scalar(200, 200, 0), 2);
        // Path prior.
        bool turn_on_prior = rrt_conf_.turn_on_prior();
            if (turn_on_prior) {
            double max_dist_sample = sqrt(Node::SquareDistance(sample, init_node))
                                    + sqrt(Node::SquareDistance(sample, goal_node));
            if (shortest_path_length_!=0 && max_dist_sample > shortest_path_length_) {
                continue;
            }
        }

        // Find parent node.
        Node nearest_node;
        if (!GetNearestNode(sample, tree, &nearest_node)) {
            continue;
        }

        // Steer.
        Node new_node;
        if (! Steer(sample, nearest_node, &new_node)) {
            continue;
        }

        // ImageProc::PlotPoint(img_env, new_node, Scalar(0, 200, 255), 3);
        // Check collision.

        if (CheckCollision(nearest_node, new_node, *environment)) {
            // Collide.
            continue;
        }
        // Add to tree.
        i = i + 1;
        new_node.SetIndex(tree.size());
        new_node.SetParent(nearest_node.index());
        tree.push_back(new_node);
        if (show_image_) {
            ImageProc::PlotLine(img_env, new_node, nearest_node,
                                Scalar(0,255,0), 1);
        }
        if (CheckTarget(new_node, environment->Goal())) {
            vector<Node> path = GetPath(tree, new_node);
            if (show_image_) {
                ImageProc::PlotPath(img_env, path, Scalar(0,0,255), 2);
            }
            double path_length = PathLength(path);
            std::cout << "A  path found" << path_length << "!" << std::endl;
            if (shortest_path_length_==0 || shortest_path_length_ > path_length) {
                std::cout << "A shorter path found!" << std::endl;
                std::cout << "path length: " << path_length << std::endl;
                for (Node node : path) {
                    std::cout << "row: " << node.row() << ", col: " << node.col()
                        << ", theta: " << node.theta() << ", id: " << node.index()
                        << ", parent:" << node.parent_index() << std::endl;
                }
                shortest_path_length_ = path_length;
            }
        }
    }

    if (show_image_)
        imshow("environment", img_env);
    return PlanningStatus::OK();
}

PlanningStatus HeuristicRRT::GetGridMap(
    const sensor_msgs::Image& image) {

    return PlanningStatus::OK();
}

bool HeuristicRRT::GetNearestNode(
        const Node& sample,
        const std::vector<Node> tree,
        Node* nearest_node) {

    Compare cmp(sample);
    std::priority_queue<Node, vector<Node>, decltype(cmp)> all_nodes(cmp);
    for (Node node : tree) {
        all_nodes.push(node);
    }
    bool success = false;
    double dtheta = 0.0;
    while (!all_nodes.empty()) {
        Node tmp = all_nodes.top();
        Node parent_tmp;
        if (tmp.parent_index() != -1) {
            parent_tmp = tree[tmp.parent_index()];
            dtheta = Node::GetDeltaTheta(parent_tmp, tmp, sample);
            if (dtheta < M_PI / 3) {
                all_nodes.pop();
            } else {
                *nearest_node = tmp;
                success = true;
                break;
            }
        } else {
            *nearest_node = tmp;
            success = true;
            break;
        }

    }
    return success;
}

bool HeuristicRRT::Steer(const Node& sample, const Node& nearest,
                         Node* new_node) {
    double theta = atan2(sample.row() - nearest.row(), sample.col() - nearest.col());
    new_node->SetTheta(theta);
    new_node->SetRow(nearest.row() + rrt_conf_.step_size() * sin(theta));
    new_node->SetCol(nearest.col() + rrt_conf_.step_size() * cos(theta));
    if (new_node->row() < planning_conf_.vrep_conf().resolutiony() &&
        new_node->col() < planning_conf_.vrep_conf().resolutionx() &&
        new_node->row() >= 0 && new_node->col() >= 0) {
            return true;
        } else {
            return false;
        }
}

// If collide, return true.
bool HeuristicRRT::CheckCollision(
        const planning::Node &a,
        const planning::Node &b,
        const Environment& env) {
    double dist = sqrt(Node::SquareDistance(a, b));
    double theta = atan2(a.row() - b.row(), a.col() - b.col());
    for (int i = 0; i <= dist; i = i + 2) {
        double row = b.row() + i * sin(theta);
        double col = b.col() + i * cos(theta);
        if (env.CheckCollisionByPixelCoord(row, col)) {
            return true;
        }
    }

    return false;
}

bool HeuristicRRT::CheckTarget(const Node& node, const Node& goal) {
    if (Node::SquareDistance(node, goal) < 400) {
        return true;
    }
    return false;
}

vector<Node> HeuristicRRT::GetPath(const std::vector<Node>& tree,
                                   const Node& new_node) {
    std::vector<Node> path = {new_node};
    int parent_index = new_node.parent_index();
    while (parent_index != -1) {
        path.insert(path.begin(), tree[parent_index]);
        parent_index = tree[parent_index].parent_index();
   }
   return path;
}

double HeuristicRRT::PathLength(const std::vector<Node>& path) {
    double length = 0;
    for (int i = 0; i < path.size()-1; ++i) {
        length += sqrt(Node::SquareDistance(path[i], path[i+1]));
    }
    return length;
}

//std::vector<Node> HeuristicRRT::PostProcessing() {

//}

}  // namespace planning
