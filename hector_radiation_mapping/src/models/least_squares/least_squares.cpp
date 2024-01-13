#include "models/least_squares/least_squares.h"
#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "hector_radiation_mapping/sampleManager.h"
#include <ceres/ceres.h>
#include "glog/logging.h"

LeastSquares::LeastSquares() : Model(ModelType::LEAST_SQUARES, Parameters::instance().ls_min_update_time) {
    layer_name_error_ = "error";
    layer_name_intensity_ = "intensity";
    layer_name_i_numerator_ = "i_numerator";
    layer_name_i_denominator_ = "i_denominator";
    grid_map_ = std::make_shared<GridMap>(Parameters::instance().ls_grid_map_topic,
                                          Parameters::instance().ls_grid_map_resolution);
    grid_map_->addLayer(layer_name_error_);
    grid_map_->addLayer(layer_name_i_numerator_);
    grid_map_->addLayer(layer_name_i_denominator_);
    grid_map_->addLayer(layer_name_intensity_);
    use_circle_ = false;
    slam_map_subscriber_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe<nav_msgs::OccupancyGrid>(
                    Parameters::instance().environment_map_topic, 1, &LeastSquares::slamMapCallback, this));

    google::InitGoogleLogging("least_squares");
}

void LeastSquares::reset() {
    std::lock_guard<std::mutex> lock{sample_queue_mtx_};
    samples_.clear();
    samples_add_queue_.clear();
    samples_delete_queue_.clear();
    samples_new_.clear();
}

void LeastSquares::update() {
    updateSamples();
    static Matrix samplePositions;
    static bool use_circle = use_circle_;
    static Vector2d position = SampleManager::instance().getLastSamplePos().topRows(2);
    {
        std::lock_guard<std::mutex> lock{grid_map_->getGridMapMutex()};
        samplePositions = grid_map_->getSamplePositions(slam_map_, use_circle, Parameters::instance().ls_circle_radius, position);
    }
    evaluate(samplePositions);
}

template<typename T>
bool LeastSquares::CostFunctor::operator()(const T *const x, T *residual) const {
    residual[0] = 10.0 - x[0];
    return true;
}

void LeastSquares::evaluate(Matrix &positions) {
    std::lock_guard<std::mutex> lock{grid_map_->getGridMapMutex()};
    grid_map::Matrix &layer_error = (grid_map_->getGridMapRef())[layer_name_error_];
    grid_map::Matrix &layer_i_numerator = (grid_map_->getGridMapRef())[layer_name_i_numerator_];
    grid_map::Matrix &layer_i_denominator = (grid_map_->getGridMapRef())[layer_name_i_denominator_];
    grid_map::Matrix &layer_intensity = (grid_map_->getGridMapRef())[layer_name_intensity_];

    for (int col = 0; col < layer_error.cols(); ++col) {
        for (int row = 0; row < layer_error.rows(); ++row) {
            int index = col * layer_error.rows() + row;
            Vector2d position = positions.row(index);
            if (std::isnan(layer_error(row, col))) {
                layer_i_numerator(row, col) = 0.0f;
                layer_i_denominator(row, col) = 0.0f;
                calculateOptimalIntensity(position, samples_, layer_intensity(row, col), layer_i_numerator(row, col),
                                          layer_i_denominator(row, col));
            } else {
                calculateOptimalIntensity(position, samples_new_, layer_intensity(row, col), layer_i_numerator(row, col),
                                          layer_i_denominator(row, col));
            }

            layer_error(row, col) = 0.0f;
            float dist2_inv;
            float error_part;
            for (const Sample &sample: samples_) {
                dist2_inv = 1.0 / (sample.get2DPos() - position).squaredNorm();
                error_part = (sample.doseRate_ - layer_intensity(row, col) * dist2_inv);
                layer_error(row, col) += error_part * error_part;
            };
        }
    }

    grid_map_->publish();
    /*
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

    //std::cout << summary.BriefReport() << "\n";
    //std::cout << "x : " << initial_x
    //          << " -> " << x << "\n";
    */
}

void LeastSquares::paramCallback() {

}

void LeastSquares::slamMapCallback(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &grid_msg_ptr) {
    slam_map_ = std::make_shared<nav_msgs::OccupancyGrid>(*grid_msg_ptr);
}

void
LeastSquares::calculateOptimalIntensity(const Vector2d &position, const std::vector<Sample> &samples, float &intensity,
                                        float &numerator, float &denominator) {
    float dist2_inv;
    float dist4_inv;
    for (const Sample &sample: samples) {
        dist2_inv = 1.0 / (sample.get2DPos() - position).squaredNorm();
        dist4_inv = dist2_inv * dist2_inv;
        numerator += sample.doseRate_ * dist2_inv;
        denominator += dist4_inv;
    };
    intensity = numerator / denominator;
}
