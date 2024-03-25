#include "models/least_squares/least_squares.h"
#include "util/parameters.h"
#include "util/util.h"
#include "util/dddynamic_reconfigure.h"
#include "hector_radiation_mapping/sampleManager.h"
#include <ceres/ceres.h>
#include "exploration/exploration_manager.h"
#include "glog/logging.h"

LeastSquares::LeastSquares() : Model(ModelType::LEAST_SQUARES, Parameters::instance().ls_on_start_up,
                                     Parameters::instance().ls_min_update_time) {
    max_queue_size_ = Parameters::instance().ls_max_queue;
    layer_name_error_ = "error";
    layer_name_error_radius_ = "error_radius";
    layer_name_error_latest_ = "error_latest";
    grid_map_ = std::make_shared<GridMap>(Parameters::instance().ls_grid_map_topic,
                                          Parameters::instance().ls_grid_map_resolution);
    grid_map_->addLayer(layer_name_error_);
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

    result_ = nullptr;
    new_result_ = nullptr;
    gradient_marker_ = ArrowMarker(Vector3d(0.0, 0.0, 0.0), Vector3d(1.0, 1.0, 0.0));
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
    evaluate_thread3.join();

    Vector2d center = SampleManager::instance().getLastSamplePos().topRows(2);
    grid_map::GridMap &grid_map_ref = grid_map_->getGridMapRef();
    for (TextMarker text_marker: text_markers_) {
        text_marker.deleteMarker();
    }

    new_result_ = std::make_shared<Result>();
    createMinMarkers(center, grid_map_ref, (grid_map_ref)[layer_name_error_latest_], {0.0, 0.0, 1.0, 1.0});
    new_result_->latest_minima.emplace_back(text_markers_.back().getPos().topRows(2));
    createMinMarkers(center, grid_map_ref, (grid_map_ref)[layer_name_error_radius_], {0.0, 1.0, 0.0, 1.0});
    new_result_->radius_minima.emplace_back(text_markers_.back().getPos().topRows(2));
    // get last text marker

    //ExplorationManager::instance().moveBase(text_markers_.back().getPos().topRows(2));

    //createMinMarkers(center, grid_map_ref, (grid_map_ref)[layer_name_error_], {1.0, 0.0, 0.0, 1.0});
    //evaluate();

    {
        std::lock_guard<std::mutex> result_lock{result_mutex_};
        result_ = new_result_;
    }
    grid_map_->publish();
}

void LeastSquares::evaluate3() {
    Clock clock;
    clock.tick();
    grid_map::GridMap &grid_map_ref = grid_map_->getGridMapRef();
    grid_map::Matrix &layer_error_radius = (grid_map_ref)[layer_name_error_radius_];
    grid_map::Matrix &layer_error_latest = (grid_map_ref)[layer_name_error_latest_];
    grid_map::Matrix &layer_error = (grid_map_ref)[layer_name_error_];
    layer_error.setConstant(std::numeric_limits<float>::quiet_NaN());
    layer_error_radius.setConstant(std::numeric_limits<float>::quiet_NaN());
    layer_error_latest.setConstant(std::numeric_limits<float>::quiet_NaN());

    // update gradient marker
    Vector2d gradient = SampleManager::instance().getRadiationGradient(SampleManager::instance().getLastSamplePos().topRows(2), 1.0);
    gradient_marker_.setDirection(Vector3d(gradient.x(), gradient.y(), 0.0), true);
    gradient_marker_.setPos(SampleManager::instance().getLastSamplePos().topRows(2).x(),
                            SampleManager::instance().getLastSamplePos().topRows(2).y(), 0.0, true);

    Vector2d center = SampleManager::instance().getLastSamplePos().topRows(2);
    std::vector<Sample> samples_in_radius = SampleManager::instance().getSamplesWithinRadius(center, 5.0);

    float numerator, denominator, intensity;
    int col, row;
    grid_map::Position position;
    grid_map::Index index;

    for (grid_map::CircleIterator it(grid_map_ref, center, 6.0); !it.isPastEnd(); ++it) {
        index = *it;
        col = index(1);
        row = index(0);
        grid_map_ref.getPosition(*it, position);
        numerator = 0.0f;
        denominator = 0.0f;
        calculateOptimalIntensity(position, samples_in_radius, intensity, numerator, denominator);
        calculateError(position, samples_in_radius, intensity, layer_error_radius(row, col));
    }

    std::vector<Sample> samples_latest = SampleManager::instance().getLatestSamples(max_queue_size_);
    for (grid_map::CircleIterator it(grid_map_ref, center, 6.0); !it.isPastEnd(); ++it) {
        index = *it;
        col = index(1);
        row = index(0);
        grid_map_ref.getPosition(*it, position);
        numerator = 0.0f;
        denominator = 0.0f;
        calculateOptimalIntensity(position, samples_latest, intensity, numerator, denominator);
        calculateError(position, samples_latest, intensity, layer_error_latest(row, col));
    }

    /*
    for (grid_map::CircleIterator it(grid_map_ref, center, 6.0); !it.isPastEnd(); ++it) {
        index = *it;
        col = index(1);
        row = index(0);
        grid_map_ref.getPosition(*it, position);
        numerator = 0.0f;
        denominator = 0.0f;
        calculateOptimalIntensity(position, samples_, intensity, numerator, denominator);
        calculateError(position, samples_, intensity, layer_error(row, col));
    }
    */

    long time = clock.tock();
    ROS_INFO_STREAM("LS2 " << time << " ms");
}

void LeastSquares::evaluate() {
    Clock clock;
    clock.tick();

    Vector2d initial_pos = text_markers_.back().getPos().topRows(2);
    std::vector<Sample> samples_in_radius = SampleManager::instance().getSamplesWithinRadius(initial_pos, 5.0);

    double initial_x = initial_pos.x();
    double initial_y = initial_pos.y();
    double initial_intensity = 1.0;

    ceres::Problem problem;
    double estimated_intensity = initial_intensity;
    double estimated_x = initial_x;
    double estimated_y = initial_y;

    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<InverseAltSquareCostFunctor, 1, 1, 1>(new InverseAltSquareCostFunctor(samples_in_radius));
    problem.AddResidualBlock(cost_function, nullptr, &estimated_x, &estimated_y);

    /*
    for(Sample sample : samples_in_radius){
        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<InverseSquareCostFunctor, 1, 1, 1, 1>(
                        new InverseSquareCostFunctor(sample.get2DPos().x(), sample.get2DPos().y(), sample.dose_rate_));
        problem.AddResidualBlock(cost_function, nullptr, &estimated_x, &estimated_y, &estimated_intensity);
    }*/

    // Set solver options
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;

    // Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "Initial guess for position: (" << initial_x << ", " << initial_y << ")\n";
    std::cout << "Final estimate for position: (" << estimated_x << ", " << estimated_y << ")\n";

    Vector3d min_pos3d = Vector3d(estimated_x, estimated_y, 0.0);
    TextMarker text_marker(min_pos3d, "CERES", 0.2, {1.0, 1.0, 0.0, 1.0});
    text_markers_.push_back(text_marker);
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
        numerator += (float) sample.dose_rate_ * dist2_inv;
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
        error_part = ((float) sample.dose_rate_ - intensity * dist2_inv);
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


    std::vector<double> angles;
    Vector2d direction;
    Vector2d average_direction = Vector2d(0.0, 0.0);

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

        direction = (min_pos-center).normalized();
        average_direction += direction;
        angles.push_back(Util::directionToAngle(direction));
    }

    double mean_angle = Util::circularMean(angles);
    double variance = Util::circularVariance(angles, mean_angle);

    average_direction.normalize();
    average_direction += center;
    Vector3d average_direction3d = Vector3d(average_direction.x(), average_direction.y(), 0.0);
    TextMarker text_marker(average_direction3d, "LS - " + std::to_string(variance), 0.2, {1.0, 1.0, 1.0, 1.0});
    text_markers_.push_back(text_marker);
}

std::shared_ptr<LeastSquares::Result> LeastSquares::getResult() {
    std::lock_guard<std::mutex> lock{result_mutex_};
    return result_;
}
