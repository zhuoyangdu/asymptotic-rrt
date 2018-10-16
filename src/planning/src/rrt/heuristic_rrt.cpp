// Copyright [2018] <Zhuoyang Du>

#include "heuristic_rrt.h"
#include <queue>
#include <functional>
#include <chrono>
#include "float.h"
using namespace std;

namespace planning {

HeuristicRRT::HeuristicRRT(const PlanningConf& planning_conf)
        : rrt_conf_(planning_conf.rrt_conf()), is_init_(true),
          planning_conf_(planning_conf) {
    // pub_map_ = private_nh_.advertise<grid_map_msgs::GridMap>(
    //             "/planning/map", 1, true);
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

    // grid_map_msgs::GridMap attractive_msg = ImageProc::ImageToGridMapMsg(
    //                                         attractive_prob);
    if (show_image_)
        imshow("attractive", attractive_prob);
    // pub_map_.publish(attractive_msg);
    ProbablisticMap probablistic_map(attractive_prob);

    Node init_node(int(init.x), int(init.y), vehicle_state.theta);
    init_node.SetIndex(0);
    init_node.SetParent(-1);

    std::vector<Node> tree = {init_node};

    Node goal_node = environment->Goal();
    std::cout << "init:" << vehicle_state.x << "," << vehicle_state.y << std::endl;
    std::cout << "init pixel:" << init_node.row() << ", " << init_node.col() << std::endl;
    std::cout << "goal pixel:" << goal_node.row() << ", " << goal_node.col() << std::endl;

    auto start = std::chrono::system_clock::now();
    int i = 0;
    while (i < rrt_conf_.max_attemp()) {
        i++;
        if ( i % 200 == 0) {
            // std::cout << "iteration " << i << " times." << std::endl;
            i ++;
        }

        // Heuristic sample.
        auto t1 = std::chrono::system_clock::now();
        Node sample = probablistic_map.Sampling();
        auto t2 = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = t2 - t1;
        std::cout << "Sampling elapsed seconds:" << elapsed_seconds.count() << "s\n";

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
        t1 = std::chrono::system_clock::now();
        Node nearest_node;
        if (!GetNearestNode(sample, tree, &nearest_node)) {
            continue;
        }
        t2 = std::chrono::system_clock::now();
        elapsed_seconds = t2 - t1;
        std::cout << "GetNearestNode elapsed seconds:" << elapsed_seconds.count() << "s\n";


        // Steer.
        Node new_node;
        t1 = std::chrono::system_clock::now();
        if (! Steer(sample, nearest_node, &new_node)) {
            continue;
        }
        t2 = std::chrono::system_clock::now();
        elapsed_seconds = t2 - t1;
        std::cout << "Steer elapsed seconds:" << elapsed_seconds.count() << "s\n";

        // ImageProc::PlotPoint(img_env, new_node, Scalar(0, 200, 255), 3);
        // Check collision.

        if (CheckCollision(nearest_node, new_node, *environment)) {
            // Collide.
            continue;
        }
        // Add to tree.
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
            // std::cout << "A  path found" << path_length << "!" << std::endl;
            if (shortest_path_length_==0 || shortest_path_length_ > path_length) {
                std::cout << "A shorter path found!" << std::endl;
                //std::cout << "path length: " << path_length << std::endl;
                for (Node node : path) {
                    //std::cout << "row: " << node.row() << ", col: " << node.col()
                    //    << ", theta: " << node.theta() << ", id: " << node.index()
                    //    << ", parent:" << node.parent_index() << std::endl;
                }
                shortest_path_length_ = path_length;
                min_path = path;
            }
        }
    }
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::cout << "elapsed seconds:" << elapsed_seconds.count() << "s\n";
    if (min_path.size()!=0) {

        std::vector<Node> spline_path = PostProcessing(min_path, environment);
    }
    Record(tree);

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

std::vector<Node> HeuristicRRT::PostProcessing(
        const std::vector<Node>& path,
        const Environment* env) {
    cv::Mat img_env;
    cvtColor(env->DynamicMap(), img_env, COLOR_GRAY2BGR);

    cv::Mat repulsive_row = env->RepulsiveX();
    cv::Mat repulsive_col = env->RepulsiveY();
    std::vector<double> x;
    std::vector<double> y;
    for (Node node : path) {
        x.push_back(node.row());
        y.push_back(node.col());
    }
    double init_a = path[0].theta();
    double y1, y0, x1, x0;

    if (init_a <= M_PI/4 && init_a >=-M_PI/4) {
        y1 = y[0] - 40;
        y0 = (5*y[0]-y1)/4;
        x1 = -2*((0.5* y[0] - 0.5*y1) * tan(init_a) - 0.5*x[0]);
        x0 = (5*x[0] - x1)/4;
    } else if (init_a >= 3*M_PI/4 || init_a <= -3*M_PI/4) {
        y1 = y[0] + 40;
        y0 = (5*y[0]-y1)/4;
        x1 = -2*((0.5* y[0] - 0.5*y1) * tan(init_a) - 0.5*x[0]);
        x0 = (5*x[0] - x1)/4;
    } else if (init_a>M_PI/4 && init_a<= 3*M_PI/4) {
        x1 = x[0] - 40;
        x0 = (5*x[0] - x1)/4;
        y1 = -2*((0.5* x[0]  - 0.5*x1)/ tan(init_a) - 0.5*y[0]);
        y0 = (5*y[0]-y1)/4;
    } else if (init_a>-3*M_PI/4 && init_a<= -M_PI/4) {
        x1 = x[0] + 40;
        x0 = (5*x[0] - x1)/4;
        y1 = -2*((0.5* x[0] - 0.5*x1)/ tan(init_a) - 0.5*y[0]);
        y0 = (5*y[0]-y1)/4;
    }

    x.insert(x.begin(), {x1, x0});
    y.insert(y.begin(), {y1, y0});
    int size = x.size();
    x.push_back(2*x[size-1] - x[size-2]);
    y.push_back(2*y[size-1] - y[size-2]);
    size = x.size();

    for (int n = 0; n < rrt_conf_.post_iteration(); ++n) {
        std::vector<double> l_phi_dx, l_phi_dy, p_phi_x, p_phi_y;
        for (int i = 0; i < size; ++i) {
            l_phi_dx.push_back(0);
            l_phi_dy.push_back(0);
            p_phi_x.push_back(0);
            p_phi_y.push_back(0);
        }
        for (int i = 2; i < size -2; ++i) {
            for (int t = 0; t <20; ++t) {
                double u  = t * 1.0 / 19.0;
                double b0 = pow(1.0-u,3)/6.0;
                double b1 = (3.0*pow(u,3)-6.0*pow(u,2)+4)/6.0;
                double b2 = (-3.0*pow(u,3)+3.0*pow(u,2)+3.0*u+1.0)/6.0;
                double b3 = pow(u,3)/6.0;

                double b0_dot = -pow(1.0-u,2)/2.0;
                double b1_dot = 1.5*pow(u,2)-2.0*u;
                double b2_dot = -1.5*pow(u,2)+u+0.5;
                double b3_dot = 0.5*pow(u,2);

                double xd = x[i-2] * b0_dot + x[i-1] * b1_dot +
                            x[i] * b2_dot + x[i+1] * b3_dot;
                double yd = y[i-2] * b0_dot + y[i-1] * b1_dot +
                            y[i] * b2_dot + y[i+1] * b3_dot;
                double xk = x[i-2] * b0 + x[i-1] * b1 +
                            x[i] * b2 + x[i+1] * b3;
                double yk = y[i-2] * b0 + y[i-1] * b1 +
                            y[i] * b2 + y[i+1] * b3;

                l_phi_dx[i-2] += 2.0 * xd * b0_dot;
                l_phi_dx[i-1] += 2.0 * xd * b1_dot;
                l_phi_dx[i]   += 2.0 * xd * b2_dot;
                l_phi_dx[i+1] += 2.0 * xd * b3_dot;
                l_phi_dy[i-2] += 2.0 * yd * b0_dot;
                l_phi_dy[i-1] += 2.0 * yd * b1_dot;
                l_phi_dy[i]   += 2.0 * yd * b2_dot;
                l_phi_dy[i+1] += 2.0 * yd * b3_dot;

                xk = xk > 511 ? 511 : xk;
                xk = xk < 0 ? 0 : xk;
                yk = yk > 511 ? 511 : yk;
                yk = yk < 0 ? 0 : yk;

                double px = repulsive_row.at<double>(xk, yk);
                double py = repulsive_col.at<double>(xk, yk);

                p_phi_x[i-2] += px * b0;
                p_phi_x[i-1] += px * b1;
                p_phi_x[i]   += px * b2;
                p_phi_x[i+1] += px * b3;
                p_phi_y[i-2] += py * b0;
                p_phi_y[i-1] += py * b1;
                p_phi_y[i]   += py * b2;
                p_phi_y[i+1] += py * b3;
            }
        }

        double s_error = 0.0;
        double p_error = 0.0;
        for (int t = 2; t < size-2; ++t) {
            s_error += fabs(l_phi_dx[t]) + fabs(l_phi_dy[t]);
            p_error += fabs(p_phi_x[t]) + fabs(p_phi_y[t]);
        }
        double error = s_error + p_error;
        double lambda = rrt_conf_.k_repulsive();
        for (int t = 3; t < size-3; ++t) {
            x[t] -= 0.01 * (1.2 * l_phi_dx[t] / 512 * 20 - lambda * p_phi_x[t]);
            y[t] -= 0.01 * (1.2 * l_phi_dy[t] / 512 * 20 - lambda * p_phi_y[t]);
        }
        // ImageProc::PlotPath(img_env, x, y, Scalar(0,255,255),1);
    }

    std::vector<Node> spline_path;
    for (int i = 2 ; i < x.size()-3; ++i) {
        spline_path.push_back(Node(x[i], y[i]));
    }

    // ImageProc::PlotPath(img_env, path, Scalar(255,0,0), 2);
    // ImageProc::PlotPath(img_env, spline_path, Scalar(0,255,0), 2);
    // imshow("path", img_env);

    return path;
}


string int2string(int value)
{
    stringstream ss;
    ss<<value;
    return ss.str();
}

void HeuristicRRT::Record(const std::vector<Node>& tree) {
    time_t t = std::time(0);
    struct tm * now = std::localtime( & t );
    string time_s;
    //the name of bag file is better to be determined by the system time
    time_s = int2string(now->tm_year + 1900)+
          '-'+int2string(now->tm_mon + 1)+
          '-'+int2string(now->tm_mday)+
          '-'+int2string(now->tm_hour)+
          '-'+int2string(now->tm_min)+
          '-'+int2string(now->tm_sec);

    std::string file_name = rrt_conf_.record_path()
                            + "/tree-" + time_s + ".txt";
    std::ofstream out_file(file_name_.c_str());
    if (!out_file) {
        ROS_INFO("no file!");
    }
    for (Node node : tree) {
        out_file >> node.index() >> "\t" >> node.row()
            >> "\t" >> node.col() >> "\t" >> node.parent_index()
            >> "\n";
    }
    out_file.close();
}

}  // namespace planning
