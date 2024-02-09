#include "models/least_squares/least_squares.h"
#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "hector_radiation_mapping/sampleManager.h"
#include <ceres/ceres.h>
#include "glog/logging.h"

LeastSquares::LeastSquares() : Model(ModelType::LEAST_SQUARES, Parameters::instance().ls_on_start_up,
                                     Parameters::instance().ls_min_update_time) {
    max_queue_size_ = Parameters::instance().ls_max_queue;
    layer_name_error_ = "error";
    layer_name_intensity_ = "intensity";
    layer_name_i_numerator_ = "i_numerator";
    layer_name_i_denominator_ = "i_denominator";
    layer_name_error_radius_ = "error_radius";
    layer_name_error_latest_ = "error_latest";
    grid_map_ = std::make_shared<GridMap>(Parameters::instance().ls_grid_map_topic,
                                          Parameters::instance().ls_grid_map_resolution);
    grid_map_->addLayer(layer_name_error_);
    grid_map_->addLayer(layer_name_i_numerator_);
    grid_map_->addLayer(layer_name_i_denominator_);
    grid_map_->addLayer(layer_name_intensity_);
    grid_map_->addLayer(layer_name_error_radius_);
    grid_map_->addLayer(layer_name_error_latest_);

    use_circle_ = false;
    slam_map_subscriber_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe<nav_msgs::OccupancyGrid>(
                    Parameters::instance().environment_map_topic, 1, &LeastSquares::slamMapCallback, this));

    DDDynamicReconfigure::instance().registerVariable<int>(getShortModelName() + "_queue_size",
                                                           max_queue_size_,
                                                           boost::bind(&LeastSquares::setMaxQueueSize, this, _1),
                                                           "min/max",
                                                           0, 2000, getShortModelName());

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
    {
        std::unique_lock<std::mutex> lock{sample_queue_mtx_};
        update_condition_.wait(lock, [this]() { return (!samples_add_queue_.empty()) || !active_; });
        if (!active_) return;
    }
    updateSamples();

    std::lock_guard<std::mutex> lock{grid_map_->getGridMapMutex()};
    if (slam_map_ != nullptr) {
        grid_map_->updateGridDimensionWithSlamMap(slam_map_);
    }

    // execute evaluate3 and evaluate in their own thread
    std::thread evaluate_thread3(&LeastSquares::evaluate3, this);
    std::thread evaluate_thread(&LeastSquares::evaluate, this);
    evaluate_thread3.join();
    evaluate_thread.join();

    Vector2d center = SampleManager::instance().getLastSamplePos().topRows(2);
    grid_map::GridMap &grid_map_ref = grid_map_->getGridMapRef();
    for (TextMarker text_marker: text_markers_) {
        text_marker.deleteMarker();
    }
    createMinMarkers(center, grid_map_ref, (grid_map_ref)[layer_name_error_radius_], {0.0, 1.0, 0.0, 1.0});
    createMinMarkers(center, grid_map_ref, (grid_map_ref)[layer_name_error_latest_], {0.0, 0.0, 1.0, 1.0});
    createMinMarkers(center, grid_map_ref, (grid_map_ref)[layer_name_error_], {1.0, 0.0, 0.0, 1.0});
    grid_map_->publish();
}

template<typename T>
bool LeastSquares::CostFunctor::operator()(const T *const x, T *residual) const {
    residual[0] = 10.0 - x[0];
    return true;
}

void LeastSquares::evaluate3() {
    Clock clock;
    clock.tick();
    grid_map::GridMap &grid_map_ref = grid_map_->getGridMapRef();
    grid_map::Matrix &layer_error_radius = (grid_map_ref)[layer_name_error_radius_];
    grid_map::Matrix &layer_error_latest = (grid_map_ref)[layer_name_error_latest_];

    u_int max_queue_size = max_queue_size_;
    Vector2d center = SampleManager::instance().getLastSamplePos().topRows(2);
    std::vector<Sample> samples_in_radius = SampleManager::instance().getSamplesWithinRadius(center, 5.0);

    float numerator, denominator, intensity;
    int col, row;
    grid_map::Position position;
    grid_map::Index index;
    for (grid_map::GridMapIterator it(grid_map_ref); !it.isPastEnd(); ++it) {
        index = *it;
        col = index(1);
        row = index(0);
        grid_map_ref.getPosition(*it, position);
        numerator = 0.0f;
        denominator = 0.0f;
        calculateOptimalIntensity(position, samples_in_radius, intensity, numerator, denominator);
        calculateError(position, samples_in_radius, intensity, layer_error_radius(row, col));
    }

    std::vector<Sample> samples_latest = samples_;
    std::sort(samples_latest.begin(), samples_latest.end(), [](const Sample &a, const Sample &b) {
        return a.time_ > b.time_;
    });
    samples_latest.resize(max_queue_size);

    for (grid_map::GridMapIterator it(grid_map_ref); !it.isPastEnd(); ++it) {
        index = *it;
        col = index(1);
        row = index(0);
        grid_map_ref.getPosition(*it, position);
        numerator = 0.0f;
        denominator = 0.0f;
        calculateOptimalIntensity(position, samples_latest, intensity, numerator, denominator);
        calculateError(position, samples_latest, intensity, layer_error_latest(row, col));
    }

    long time = clock.tock();
    ROS_INFO_STREAM("LS2 " << time << " ms");
}

void LeastSquares::evaluate() {
    Clock clock;
    clock.tick();
    grid_map::GridMap &grid_map_ref = grid_map_->getGridMapRef();
    grid_map::Matrix &layer_error = (grid_map_ref)[layer_name_error_];
    grid_map::Matrix &layer_i_numerator = (grid_map_ref)[layer_name_i_numerator_];
    grid_map::Matrix &layer_i_denominator = (grid_map_ref)[layer_name_i_denominator_];
    grid_map::Matrix &layer_intensity = (grid_map_ref)[layer_name_intensity_];

    int col, row;
    grid_map::Position position;
    grid_map::Index index;
    for (grid_map::GridMapIterator it(grid_map_ref); !it.isPastEnd(); ++it) {
        index = *it;
        col = index(1);
        row = index(0);
        grid_map_ref.getPosition(*it, position);

        if (std::isnan(layer_error(row, col))) {
            layer_i_numerator(row, col) = 0.0f;
            layer_i_denominator(row, col) = 0.0f;
            calculateOptimalIntensity(position, samples_, layer_intensity(row, col), layer_i_numerator(row, col),
                                      layer_i_denominator(row, col));
        } else {
            calculateOptimalIntensity(position, samples_new_, layer_intensity(row, col),
                                      layer_i_numerator(row, col),
                                      layer_i_denominator(row, col));
        }
        calculateError(position, samples_, layer_intensity(row, col), layer_error(row, col));
    }

    long time = clock.tock();
    ROS_INFO_STREAM("LS1 " << time << " ms");
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

void LeastSquares::setMaxQueueSize(int max_queue_size) {
    max_queue_size_ = max_queue_size;
}

void
LeastSquares::calculateOptimalIntensity(const Vector2d &position, const std::vector<Sample> &samples, float &intensity,
                                        float &numerator, float &denominator) {
    float dist2_inv;
    float dist4_inv;
    for (const Sample &sample: samples) {
        dist2_inv = (float) (1.0 / (sample.get2DPos() - position).squaredNorm());
        dist4_inv = dist2_inv * dist2_inv;
        numerator += (float) sample.doseRate_ * dist2_inv;
        denominator += dist4_inv;
    };
    intensity = numerator / denominator;
}

void LeastSquares::calculateError(const Vector2d &position, const std::vector<Sample> &samples, float intensity,
                                  float &error) {
    float dist2_inv, error_part;
    error = 0.0f;
    for (const Sample &sample: samples) {
        dist2_inv = (float) (1.0 / (sample.get2DPos() - position).squaredNorm());
        error_part = ((float) sample.doseRate_ - intensity * dist2_inv);
        error += error_part * error_part;
    }
}

void LeastSquares::createMinMarkers(const Vector2d &center, const grid_map::GridMap &grid_map_ref,
                                    const grid_map::Matrix &layer_error, const Eigen::Vector4d &color) {
    float radius[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
    int step_size = 5;
    grid_map::Index index1;
    grid_map::Index index2;
    bool is_inside1, is_inside2;

    for (float r: radius) {
        Vector2d min_pos;
        // init min_error with max possible float value
        float min_error = std::numeric_limits<float>::max();

        for (int angle = 0; angle < 180; angle += step_size) {
            Vector2d direction = Vector2d(cos(angle * M_PI / 180.0), sin(angle * M_PI / 180.0));
            Vector2d circle_pos = center + direction * r;
            Vector2d circle_pos2 = center - direction * r;
            is_inside1 = grid_map_ref.getIndex(circle_pos, index1);
            is_inside2 = grid_map_ref.getIndex(circle_pos2, index2);
            if (is_inside1 && layer_error(index1(0), index1(1)) < min_error) {
                min_error = layer_error(index1(0), index1(1));
                min_pos = circle_pos;
            }
            if (is_inside2 && layer_error(index2(0), index2(1)) < min_error) {
                min_error = layer_error(index2(0), index2(1));
                min_pos = circle_pos2;
            }
        }

        Vector3d min_pos3d = Vector3d(min_pos(0), min_pos(1), 0.0);
        TextMarker text_marker(min_pos3d, "LS - " + std::to_string(r), 0.2, color);
        text_markers_.push_back(text_marker);
    }
}
