// Copyright [2018] <Zhuoyang Du>

#include "heuristic_rrt.h"

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
    ImageProc::PlotPoint(img_env, init, Scalar(0, 100, 0));


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

    Node init_node(int(init.x), int(init.y));
    cv::Point goal = environment->Goal();
    Node goal_node(int(goal.x), int(goal.y));
    std::cout << "init:" << vehicle_state.x << "," << vehicle_state.y << std::endl;
    std::cout << "init pixel:" << init_node.row() << ", " << init_node.col() << std::endl;
    std::cout << "goal pixel:" << goal_node.row() << ", " << goal_node.col() << std::endl;

    for (int i = 0; i < rrt_conf_.max_attemp(); ++i) {
        // Heuristic sample.
        Node sample = probablistic_map.Sampling();
        std::cout << "sample:" << sample.row() << "," << sample.col() << std::endl;
        ImageProc::PlotPoint(img_env, cv::Point(sample.col(), sample.row()), Scalar(150, 0, 0));

        // Find parent node.

    }

    ImageProc::PlotPoint(img_env, cv::Point(100,200), Scalar(150, 0, 0));

    if (show_image_)
        imshow("environment", img_env);
    return PlanningStatus::OK();
}

PlanningStatus HeuristicRRT::GetGridMap(
    const sensor_msgs::Image& image) {

    return PlanningStatus::OK();
}

Node HeuristicRRT::HeuristicSample(const cv::Mat& attractive_prob) {

}


}  // namespace planning
