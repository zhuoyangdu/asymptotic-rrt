// Copyright [2018] <Zhuoyang Du>

#include "heuristic_rrt.h"
#include <queue>
#include <functional>

using namespace std;

namespace planning {

HeuristicRRT::HeuristicRRT(const RRTConf& rrt_conf)
        : rrt_conf_(rrt_conf), is_init_(true) {
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
    std::vector<Node> tree = {init_node};

    cv::Point goal = environment->Goal();
    Node goal_node(int(goal.x), int(goal.y));
    std::cout << "init:" << vehicle_state.x << "," << vehicle_state.y << std::endl;
    std::cout << "init pixel:" << init_node.row() << ", " << init_node.col() << std::endl;
    std::cout << "goal pixel:" << goal_node.row() << ", " << goal_node.col() << std::endl;

    for (int i = 0; i < rrt_conf_.max_attemp(); ++i) {
        // Heuristic sample.
        Node sample = probablistic_map.Sampling();
        std::cout << "sample:" << sample.row() << "," << sample.col() << std::endl;
        ImageProc::PlotPoint(img_env, cv::Point(sample.col(), sample.row()),
                             Scalar(200, 200, 0), 2);

        // Find parent node.
        Node nearest_node;
        if (GetNearestNode(sample, tree, &nearest_node)) {
            // ImageProc::PlotLine(img_env, sample, nearest_node,
            //                    Scalar(0,0,255), 2);
        } else {
            std::cout << "No nearest node!" << std::endl;
            continue;
        }

        // Steer.
        Node new_node;
        Steer(sample, nearest_node, &new_node);
        // ImageProc::PlotPoint(img_env, new_node, Scalar(0, 200, 255), 3);

        // Check collision.
        if (CheckCollision(nearest_node, new_node, *environment)) {
            // Collide.
            continue;
        }

        // Add to tree.
        tree.push_back(new_node);
        ImageProc::PlotLine(img_env, new_node, nearest_node,
                            Scalar(0,255,0), 1);

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
        cout << "test:" << all_nodes.top().row() << ", " << all_nodes.top().col()
             << ",s:" << Node::SquareDistance(all_nodes.top(), sample) << endl;

        dtheta = Node::GetDeltaTheta(all_nodes.top(), sample);
        cout << "dtheta:" << dtheta << std::endl;
        if (dtheta < M_PI / 3) {
            all_nodes.pop();
        } else {
            *nearest_node = all_nodes.top();
            success = true;
            break;
        }
    }
    if (success) {
        std::cout << "nearest node: " << nearest_node->row() << "," << nearest_node->col()
            << ", dtheta: " << dtheta << std::endl;
    }
    return success;
}

void HeuristicRRT::Steer(const Node& sample, const Node& nearest,
                         Node* new_node) {
    double theta = atan2(sample.row() - nearest.row(), sample.col() - nearest.col());
    new_node->SetTheta(theta);
    new_node->SetRow(nearest.row() + rrt_conf_.step_size() * sin(theta));
    new_node->SetCol(nearest.col() + rrt_conf_.step_size() * cos(theta));
}

// If collide, return true.
bool HeuristicRRT::CheckCollision(
        const planning::Node &a,
        const planning::Node &b,
        const Environment& env) {
    double dist = sqrt(Node::SquareDistance(a, b));
    double theta = atan2(a.row() - b.row(), a.col() - b.col());
    for (int i = 0; i <= dist; i = i + 2) {
        double row = b.row() + 2.0 * sin(theta);
        double col = b.col() + 2.0 * cos(theta);
        if (env.CheckCollisionByPixelCoord(row, col)) {
            return true;
        }
    }
    return false;
}

}  // namespace planning
