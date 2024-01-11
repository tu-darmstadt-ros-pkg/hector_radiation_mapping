#include "models/least_squares/least_squares.h"
#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "hector_radiation_mapping/sampleManager.h"
#include <ceres/ceres.h>
#include "glog/logging.h"

LeastSquares::LeastSquares() : Model(ModelType::LEAST_SQUARES, Parameters::instance().ls_min_update_time) {
    layer_name_mean_ = "mean";
    grid_map_ = std::make_shared<GridMap>(Parameters::instance().ls_grid_map_topic,
                                          Parameters::instance().ls_grid_map_resolution);
    grid_map_->addLayer(layer_name_mean_);
    use_circle_ = true;
    slam_map_subscriber_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe<nav_msgs::OccupancyGrid>(
                    Parameters::instance().environment_map_topic, 1, &LeastSquares::slamMapCallback, this));

    google::InitGoogleLogging("least_squares");
}

void LeastSquares::reset() {

}

void LeastSquares::update() {
    Matrix samplePositions;
    bool use_circle = use_circle_;
    Vector2d center = SampleManager::instance().getLastSamplePos().topRows(2);
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map = slam_map_;
    {
        std::lock_guard<std::mutex> lock{grid_map_->getGridMapMutex()};
        samplePositions = grid_map_->getSamplePositions(slam_map, use_circle, Parameters::instance().ls_circle_radius, center);
    }
    evaluate(samplePositions);
}

template<typename T>
bool LeastSquares::CostFunctor::operator()(const T *const x, T *residual) const {
    residual[0] = 10.0 - x[0];
    return true;
}

void LeastSquares::evaluate(Matrix &positions) {

    double initial_x = 5.0;
    double x = initial_x;

    // Build the problem.
    ceres::Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, nullptr, &x);

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
              << " -> " << x << "\n";
}

void LeastSquares::paramCallback() {

}

void LeastSquares::slamMapCallback(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &grid_msg_ptr) {
    slam_map_ = std::make_shared<nav_msgs::OccupancyGrid>(*grid_msg_ptr);
}
