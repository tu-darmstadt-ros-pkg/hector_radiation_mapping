#include "models/least_squares/least_squares.h"
#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "hector_radiation_mapping/sampleManager.h"
#include <ceres/ceres.h>
#include "glog/logging.h"

LeastSquares::LeastSquares() : Model(ModelType::LEAST_SQUARES, Parameters::instance().ls_on_start_up, Parameters::instance().ls_min_update_time) {
    max_dist = 0.0;
    max_num = 0.0;
    max_den = 0.0;
    max_int = 0.0;
    min_den = 1000.0;
    min_int = 1000.0;
    min_dist = 1000.0;
    min_num = 1000.0;
    min_err = 1000.0;
    max_err = 0.0;
    current_queue_id_ = 0;
    max_queue_ = Parameters::instance().ls_max_queue;
    layer_name_error_ = "error";
    layer_name_intensity_ = "intensity";
    layer_name_i_numerator_ = "i_numerator";
    layer_name_i_denominator_ = "i_denominator";
    layer_name_i_nnumerator_ = "i_nnumerator";
    layer_name_i_ndenominator_ = "i_ndenominator";
    grid_map_ = std::make_shared<GridMap>(Parameters::instance().ls_grid_map_topic,
                                          Parameters::instance().ls_grid_map_resolution);
    grid_map_->addLayer(layer_name_error_);
    grid_map_->addLayer(layer_name_i_numerator_);
    grid_map_->addLayer(layer_name_i_denominator_);
    grid_map_->addLayer(layer_name_intensity_);

    grid_map2_ = std::make_shared<GridMap>(Parameters::instance().ls_grid_map_topic + "2",
                                           Parameters::instance().ls_grid_map_resolution);
    grid_map2_->addLayer(layer_name_error_);
    grid_map2_->addLayer(layer_name_i_numerator_);
    grid_map2_->addLayer(layer_name_i_denominator_);
    grid_map2_->addLayer(layer_name_intensity_);
    grid_map2_->addLayer(layer_name_i_nnumerator_);
    grid_map2_->addLayer(layer_name_i_ndenominator_);

    use_circle_ = false;
    slam_map_subscriber_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe<nav_msgs::OccupancyGrid>(
                    Parameters::instance().environment_map_topic, 1, &LeastSquares::slamMapCallback, this));

    DDDynamicReconfigure::instance().registerVariable<int>(getShortModelName() + "_queue_size",
                                                           max_queue_,
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
        update_condition_.wait(lock,[this]() { return (!samples_add_queue_.empty()) || !active_; });
        if (!active_) return;
    }
    updateSamples();
    static Matrix samplePositions;
    static bool use_circle = use_circle_;
    static Vector2d position = SampleManager::instance().getLastSamplePos().topRows(2);
    {
        std::lock_guard<std::mutex> lock{grid_map_->getGridMapMutex()};
        samplesToSamplesLS(samples_new_);
    }

    // create thread to evaluate
    evaluate3();
}

template<typename T>
bool LeastSquares::CostFunctor::operator()(const T *const x, T *residual) const {
    residual[0] = 10.0 - x[0];
    return true;
}

void LeastSquares::evaluate2() {
    Clock clock;
    clock.tick();
    std::lock_guard<std::mutex> lock{grid_map2_->getGridMapMutex()};
    if (slam_map_ != nullptr) {
        grid_map2_->updateGridDimensionWithSlamMap(slam_map_);
    }

    static Vector2d center;
    static double radius;
    static grid_map::GridMap &grid_map_ref = grid_map2_->getGridMapRef();
    grid_map::Matrix &layer_error = (grid_map_ref)[layer_name_error_];
    grid_map::Matrix &layer_i_numerator = (grid_map_ref)[layer_name_i_numerator_];
    grid_map::Matrix &layer_i_denominator = (grid_map_ref)[layer_name_i_denominator_];
    grid_map::Matrix &layer_intensity = (grid_map_ref)[layer_name_intensity_];
    int max_queue_size = max_queue_;
    for (SampleLS &sample: samples_ls_) {
        bool remove = false;
        bool add = false;
        if (sample.queue_id_ + max_queue_size > current_queue_id_) {
            if (sample.active_) {
                // sample is already in the map
                continue;
            } else {
                // add sample to map
                sample.active_ = true;
                add = true;
            }
        } else {
            if (sample.active_) {
                // delete sample, it's too old
                sample.active_ = false;
                remove = true;
            } else {
                continue;
            }
        }

        center = sample.sample.get2DPos();
        radius = std::round(
                sample.sample.effective_radius_); // max distance where doseRate is > 0.1 with inverse square law

        for (grid_map::CircleIterator iterator(grid_map_ref, center, radius); !iterator.isPastEnd(); ++iterator) {
            const grid_map::Index &index = *iterator;
            grid_map::Position position;
            grid_map_ref.getPosition(index, position);
            int col = index(1);
            int row = index(0);
            //ROS_INFO_STREAM(layer_i_numerator(row, col));
            //layer_i_numerator(row, col) = 0.0f;

            float dist2_inv = 1.0 / (center - position).squaredNorm();
            float dist4_inv = dist2_inv * dist2_inv;
            if (std::isnan(layer_i_numerator(row, col))) {
                layer_i_numerator(row, col) = 0.0f;
                layer_i_denominator(row, col) = 0.0f;
            }

            if (remove) {
                layer_i_numerator(row, col) -= sample.sample.doseRate_ * dist2_inv;
                layer_i_denominator(row, col) -= dist4_inv;
            }
            if (add) {
                layer_i_numerator(row, col) += sample.sample.doseRate_ * dist2_inv;
                layer_i_denominator(row, col) += dist4_inv;
            }
        }
    }

    for (int col = 0; col < layer_error.cols(); ++col) {
        for (int row = 0; row < layer_error.rows(); ++row) {
            if (std::isnan(layer_i_numerator(row, col))) {
                continue;
            }
            layer_intensity(row, col) = layer_i_numerator(row, col) / layer_i_denominator(row, col);
        }
    }

    layer_error = Eigen::MatrixXf::Constant(layer_error.rows(), layer_error.cols(),
                                            std::numeric_limits<float>::quiet_NaN());
    for (SampleLS &sample: samples_ls_) {
        if (!sample.active_) {
            continue;
        }
        center = sample.sample.get2DPos();
        radius = std::round(
                sample.sample.effective_radius_); // max distance where doseRate is > 0.1 with inverse square law

        for (grid_map::CircleIterator iterator(grid_map_ref, center, radius); !iterator.isPastEnd(); ++iterator) {
            const grid_map::Index &index = *iterator;
            int col = index(1);
            int row = index(0);
            if (std::isnan(layer_intensity(row, col))) {
                continue;
            }
            if (std::isnan(layer_error(row, col))) {
                layer_error(row, col) = 0.0f;
            }
            grid_map::Position position;
            grid_map_ref.getPosition(index, position);
            float dist2_inv = 1.0 / (center - position).squaredNorm();
            float error_part = (sample.sample.doseRate_ - layer_intensity(row, col) * dist2_inv);
            layer_error(row, col) += error_part * error_part;
        }
    }

    grid_map2_->publish();

    long time = clock.tock();
    ROS_INFO_STREAM("Least Squares2222 evaluation took " << time << " ms");

}


void LeastSquares::evaluate3() {
    Clock clock;
    clock.tick();
    std::lock_guard<std::mutex> lock{grid_map2_->getGridMapMutex()};
    if (slam_map_ != nullptr) {
        grid_map2_->updateGridDimensionWithSlamMap(slam_map_);
    }
    static grid_map::GridMap &grid_map_ref = grid_map2_->getGridMapRef();
    grid_map::Matrix &layer_error = (grid_map_ref)[layer_name_error_];
    grid_map::Matrix &layer_i_numerator = (grid_map_ref)[layer_name_i_numerator_];
    grid_map::Matrix &layer_i_denominator = (grid_map_ref)[layer_name_i_denominator_];
    grid_map::Matrix &layer_intensity = (grid_map_ref)[layer_name_intensity_];
    grid_map::Matrix &layer_i_nnumerator = (grid_map_ref)[layer_name_i_nnumerator_];
    grid_map::Matrix &layer_i_ndenominator = (grid_map_ref)[layer_name_i_ndenominator_];

    int max_queue_size = max_queue_;
    std::vector<SampleLS> samples_active;
    std::vector<SampleLS> samples_remove;
    std::vector<SampleLS> samples_add;

    Vector2d center =SampleManager::instance().getLastSamplePos().topRows(2);
    std::vector<Sample> samples_n = SampleManager::instance().getSamplesWithinRadius(center, 5.0);
    for (Sample &sample: samples_n) {
        samples_active.emplace_back(sample, current_queue_id_);
        current_queue_id_++;
    }

    /*
    for (SampleLS &sample: samples_ls_) {
        if (sample.queue_id_ + max_queue_size > current_queue_id_) {
            samples_active.emplace_back(sample);
            if (!sample.active_) {
                samples_add.emplace_back(sample);
                sample.active_ = true;
            }
        } else {
            if (sample.active_) {
                samples_remove.emplace_back(sample);
                sample.active_ = false;
            }
        }
    }
    */

    for (grid_map::GridMapIterator it(grid_map_ref); !it.isPastEnd(); ++it) {
        const grid_map::Index &index = *it;
        int col = index(1);
        int row = index(0);
        grid_map::Position position;
        grid_map_ref.getPosition(*it, position);

        /*
        if (std::isnan(layer_i_numerator(row, col))) {
            layer_i_numerator(row, col) = 0.0f;
            layer_i_denominator(row, col) = 0.0f;
            layer_i_nnumerator(row, col) = 0.0f;
            layer_i_ndenominator(row, col) = 0.0f;
            calculateOptimalIntensity(position, samples_active, layer_intensity(row, col), layer_i_numerator(row, col),
                                      layer_i_denominator(row, col), true);
        } else {
            calculateOptimalIntensity(position, samples_add, samples_remove, layer_intensity(row, col),
                                      layer_i_numerator(row, col),
                                      layer_i_denominator(row, col), layer_i_nnumerator(row, col), layer_i_ndenominator(row, col));
        }
        */
        layer_i_numerator(row, col) = 0.0f;
        layer_i_denominator(row, col) = 0.0f;
        layer_i_nnumerator(row, col) = 0.0f;
        layer_i_ndenominator(row, col) = 0.0f;
        calculateOptimalIntensity(position, samples_active, layer_intensity(row, col), layer_i_numerator(row, col),
                                  layer_i_denominator(row, col), true);

        layer_error(row, col) = 0.0f;
        float dist2_inv;
        float error_part;
        for (const SampleLS &sample: samples_active) {
            dist2_inv = 1.0 / (sample.sample.get2DPos() - position).squaredNorm();
            error_part = (sample.sample.doseRate_ - layer_intensity(row, col) * dist2_inv);
            layer_error(row, col) += error_part * error_part;
        };
    }
    grid_map2_->publish();
    long time = clock.tock();
    ROS_INFO_STREAM("Least Squares evaluation took " << time << " ms");
}


void LeastSquares::evaluate(Matrix &positions) {
    Clock clock;
    clock.tick();
    std::lock_guard<std::mutex> lock{grid_map_->getGridMapMutex()};
    if (slam_map_ != nullptr) {
        grid_map_->updateGridDimensionWithSlamMap(slam_map_);
    }
    static grid_map::GridMap &grid_map_ref = grid_map_->getGridMapRef();
    grid_map::Matrix &layer_error = (grid_map_ref)[layer_name_error_];
    grid_map::Matrix &layer_i_numerator = (grid_map_ref)[layer_name_i_numerator_];
    grid_map::Matrix &layer_i_denominator = (grid_map_ref)[layer_name_i_denominator_];
    grid_map::Matrix &layer_intensity = (grid_map_ref)[layer_name_intensity_];

    for (grid_map::GridMapIterator it(grid_map_ref); !it.isPastEnd(); ++it) {
        const grid_map::Index &index = *it;
        int col = index(1);
        int row = index(0);
        grid_map::Position position;
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

        layer_error(row, col) = 0.0f;
        float dist2_inv;
        float error_part;
        for (const Sample &sample: samples_) {
            dist2_inv = 1.0 / (sample.get2DPos() - position).squaredNorm();
            error_part = (sample.doseRate_ - layer_intensity(row, col) * dist2_inv);
            layer_error(row, col) += error_part * error_part;
        };
    }

    grid_map_->publish();
    long time = clock.tock();
    ROS_INFO_STREAM("Least Squares evaluation took " << time << " ms");
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
                                        float &numerator, float &denominator, bool add) {
    float dist2_inv;
    float dist4_inv;
    for (const Sample &sample: samples) {
        dist2_inv = 1.0 / (sample.get2DPos() - position).squaredNorm();
        dist4_inv = dist2_inv * dist2_inv;

        if (add) {
            numerator += sample.doseRate_ * dist2_inv;
            denominator += dist4_inv;
        } else {
            numerator -= sample.doseRate_ * dist2_inv;
            denominator -= dist4_inv;
        }
    };
    intensity = numerator / denominator;
}

void LeastSquares::samplesToSamplesLS(std::vector<Sample> &samples) {
    std::lock_guard<std::mutex> lock{sample_queue_mtx_};
    for (Sample &sample: samples) {
        samples_ls_.emplace_back(sample, current_queue_id_);
        current_queue_id_++;
    }
}

void LeastSquares::setMaxQueueSize(int max_queue_size) {
    max_queue_ = max_queue_size;
}

void LeastSquares::calculateOptimalIntensity(const Vector2d &position, const std::vector<SampleLS> &samples,
                                             float &intensity, float &numerator, float &denominator, bool add) {
    float dist2_inv;
    float dist4_inv;
    for (const SampleLS &sample: samples) {
        dist2_inv = 1.0 / (sample.sample.get2DPos() - position).squaredNorm();
        dist4_inv = dist2_inv * dist2_inv;

        if (add) {
            numerator += sample.sample.doseRate_ * dist2_inv;
            denominator += dist4_inv;
        } else {
            numerator -= sample.sample.doseRate_ * dist2_inv;
            denominator -= dist4_inv;
        }
    };
    intensity = numerator / denominator;
}

void LeastSquares::calculateOptimalIntensity(const Vector2d &position, const std::vector<SampleLS> &samples_add,
                                             const std::vector<SampleLS> &samples_remove, float &intensity,
                                             float &numerator, float &denominator, float &nnumerator,
                                             float &ndenominator) {
    double dist2;
    double dist2_inv;
    double dist4_inv;
    double pnumerator_total = 0.0;
    double pdenominator_total = 0.0;
    double nnumerator_total = 0.0;
    double ndenominator_total = 0.0;

    for (const SampleLS &sample: samples_add) {
        dist2 = (sample.sample.get2DPos() - position).squaredNorm();
        dist2_inv = 1.0 / dist2;
        dist4_inv = dist2_inv * dist2_inv;
        pnumerator_total += sample.sample.doseRate_ * dist2_inv;
        pdenominator_total += dist4_inv;
    }
    for (const SampleLS &sample: samples_remove) {
        dist2 = (sample.sample.get2DPos() - position).squaredNorm();
        dist2_inv = 1.0 / dist2;
        dist4_inv = dist2_inv * dist2_inv;
        nnumerator_total += sample.sample.doseRate_ * dist2_inv;
        ndenominator_total += dist4_inv;
    }
    numerator += (float) pnumerator_total;
    denominator += (float) pdenominator_total;
    nnumerator += (float) nnumerator_total;
    ndenominator += (float) ndenominator_total;
    intensity = (numerator - nnumerator) / (denominator - ndenominator);
}
