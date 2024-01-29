#include "models/gpython/gaussian_process_2d.h"
#include "models/gpython/gaussian_process_3d.h"
#include "models/model_exporter.h"
#include "hector_radiation_mapping/sampleManager.h"
#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "util/util.h"
#include "util/clock_cpu.h"

GaussianProcess2D::GaussianProcess2D() {
    group_name_ = GaussianProcess::instance().getShortModelName();
    std::string prefix = group_name_ + "2D";
    layer_name_mean_ = "prediction";
    layer_name_std_dev_ = "std_dev";
    grid_map_ = std::make_shared<GridMap>(Parameters::instance().gp_grid_map_topic, Parameters::instance().gp_grid_map_resolution);
    grid_map_->addLayer(layer_name_mean_);
    grid_map_->addLayer(layer_name_std_dev_);
    use_circle_ = true;
    do_evaluation_ = false;
    min_update_time_ = Parameters::instance().gp_min_update_time_2d;
    DDDynamicReconfigure::instance().registerVariable<bool>(prefix + "_useCircleEvaluation", use_circle_,
                                                            boost::bind(&GaussianProcess2D::setUseCircle, this, _1), "on/off",
                                                            false, true, group_name_);
    DDDynamicReconfigure::instance().registerVariable<int>(prefix + "_minUpdateTime", min_update_time_,
                                                           boost::bind(&GaussianProcess2D::setMinUpdateTime, this, _1),
                                                           "min/max",
                                                           0, 5000, group_name_);
    DDDynamicReconfigure::instance().publish();

    slam_map_subscriber_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe(Parameters::instance().environment_map_topic,
                                                               Parameters::instance().dose_sub_size,
                                                               &GaussianProcess2D::slamMapCallback, this));

    active_ = false;
}

GaussianProcess2D &GaussianProcess2D::instance() {
    static GaussianProcess2D instance;
    return instance;
}

void GaussianProcess2D::shutDown() {
    deactivate();
}

void GaussianProcess2D::activate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (active_) return;
    this->active_ = true;
    update_thread_ = std::thread(&GaussianProcess2D::updateLoop, this);
    STREAM_DEBUG("GaussianProcess2D activated");
}

void GaussianProcess2D::deactivate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (!active_) return;
    this->active_ = false;
    update_thread_.join();
    STREAM_DEBUG("GaussianProcess2D deactivated");
}

void GaussianProcess2D::updateLoop() {
    Clock clock;
    std::vector<double> evaluation_times;
    std::vector<double> source_pred_times;
    std::vector<double> evaluation_sizes;
    while (active_ && ros::ok()) {
        clock.tick();
        if (do_evaluation_) {
            do_evaluation_ = false;
            bool use_circle = use_circle_;
            Vector2d center = SampleManager::instance().getLastSamplePos().topRows(2);
            std::shared_ptr<nav_msgs::OccupancyGrid> slamMap = slam_map_;
            {
                std::lock_guard<std::recursive_mutex> lock1{GaussianProcess::instance().getModelMutex()};
                std::lock_guard<std::mutex> lock{grid_map_->getGridMapMutex()};
                Matrix sample_positions = grid_map_->getSamplePositions(slamMap, use_circle, Parameters::instance().gp_circle_radius, center);
                GaussianProcess::GPResult gp_result = GaussianProcess::instance().evaluate(sample_positions);
                updateMap(gp_result.mean, gp_result.std_dev, use_circle, center);
                evaluation_times.push_back((double) clock.tock());
                clock.tick();
                updateSourcePrediction(sample_positions, gp_result.mean);
                source_pred_times.push_back((double) clock.tock());
            }
            evaluation_sizes.push_back((double) GaussianProcess::instance().getSampleIds2d().size());
        }
        int sleepTime = std::max(0, min_update_time_ - (int) clock.tock());
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
    }
    //std::string export_path = Util::getExportPath("runtime");
    //Util::exportVectorToTxtFile(evaluation_times, export_path, "evaluation_times", Util::TxtExportType::NEW, true);
    //Util::exportVectorToTxtFile(source_pred_times, export_path, "source_pred_times", Util::TxtExportType::NEW, true);
    //Util::exportVectorToTxtFile(evaluation_sizes, export_path, "evaluation_sizes", Util::TxtExportType::NEW, true);
}

void GaussianProcess2D::updateMap(const Vector &mean, const Vector &std_dev, bool use_circle, const Vector2d &center) {
    double radius = Parameters::instance().gp_circle_radius;
    if (use_circle) {
        grid_map_->updateLayer(layer_name_mean_, mean, center, radius);
        grid_map_->updateLayer(layer_name_std_dev_, std_dev, center, radius);
    } else {
        grid_map_->updateLayer(layer_name_mean_, mean);
        grid_map_->updateLayer(layer_name_std_dev_, std_dev);
    }
    grid_map_->publish();
}

std::pair<grid_map::GridMap, std::shared_ptr<nav_msgs::OccupancyGrid>> GaussianProcess2D::getExportGridMap() {
    std::lock_guard<std::mutex> lock{grid_map_->getGridMapMutex()};
    std::lock_guard<std::recursive_mutex> lock1{GaussianProcess::instance().getModelMutex()};
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map = slam_map_;
    Matrix sample_positions = grid_map_->getSamplePositions(slam_map);
    GaussianProcess::GPResult gp_result = GaussianProcess::instance().evaluate(sample_positions);
    updateMap(gp_result.mean, gp_result.std_dev);
    return {grid_map_->getGridMap(), slam_map};
}

void GaussianProcess2D::updateSourcePrediction(const Matrix &positions, const Vector &predictions) {
    int max_optimizer_steps = 1000;
    double min_source_distance = 2 * grid_map_->getResolution();
    int dim = 2;

    // Update all existing sources (check if they are still a maximum)
    Vector dir(dim);
    Vector dirZero = Vector::Zero(dim);
    auto it = sources_.begin();
    while (it != sources_.end()) {
        dir = computeGradientStep(it->get()->getPos());
        if (dir.isApprox(dirZero)) {
            ++it;
        } else {
            if (!it->get()->isConfirmed()) {
                sources_.erase(it);
            } else {
                ++it;
            }
        }
    }
    STREAM_DEBUG("GP2D Kept " << sources_.size() << " sources.");

    // Generate start positions (All points in trajectory with at least min_dist to each other)
    Eigen::Matrix3Xd start_positions;
    Vector3d last_pos;
    Vector3d pos_3d;
    last_pos << -DBL_MAX, -DBL_MAX, -DBL_MAX;
    double mean = 0;
    double min_dist = 1;
    double dist;

    std::vector<Sample> samples = SampleManager::instance().getSamples();
    for (const Sample &sample: samples) {
        pos_3d = sample.position_;
        if (!grid_map_->isInside(pos_3d.topRows(dim))) {
            continue;
        }
        dist = (pos_3d - last_pos).norm();
        if (dist > min_dist) {
            start_positions.conservativeResize(3, start_positions.cols() + 1);
            start_positions.col(start_positions.cols() - 1) = pos_3d;
            last_pos = pos_3d;
            mean += grid_map_->atPosition("prediction", pos_3d.topRows(dim));
        }
    }
    mean = mean / start_positions.cols();
    STREAM_DEBUG("GP2D Start positions: " << start_positions.cols() << " Mean: " << mean);

    // Gradient ascent for each start position
    int steps;
    double dosage = 0.0;
    bool too_close;

    for (int i = 0; i < start_positions.cols(); i++) {
        pos_3d = start_positions.col(i);
        dir = computeGradientStep(pos_3d);

        steps = 0;
        while (!dir.isApprox(dirZero) && steps < max_optimizer_steps) {
            dir = computeGradientStep(pos_3d);
            pos_3d.topRows(dim) += dir * grid_map_->getResolution();
            steps += 1;
            dosage = grid_map_->atPosition("prediction", pos_3d.topRows(dim));
        }

        too_close = false;
        for (std::shared_ptr<SourceInteractive> &source: sources_) {
            if ((source->getPos() - pos_3d).topRows(dim).squaredNorm() < min_source_distance * min_source_distance) {
                too_close = true;
                break;
            }
        }

        if (!too_close && dosage > Parameters::instance().gp_mean_factor * mean) {
            if (dosage < 0.1) {
                continue;
            }
            STREAM_DEBUG("GP2D Adding source: ");
            Vector3d pos = Vector3d(pos_3d.x(), pos_3d.y(), 0.0);
            std::shared_ptr<SourceInteractive> source = std::make_shared<SourceInteractive>(pos, dosage, dosage,
                                                                                            boost::bind(
                                                                                                    &GaussianProcess2D::interactiveMarkerCallback,
                                                                                                    this, _1));
            sources_.push_back(source);
            STREAM_DEBUG("GP2D Adding source: " + std::to_string(source->getStrength()));
        }
    }
    STREAM_DEBUG("Done with sources");
}

Vector GaussianProcess2D::computeGradientStep(const Vector &position) {
    int dim = 2;
    Vector direction = Vector::Zero(dim);
    Vector pos = position.topRows(dim);

    if (!grid_map_->isInside(pos) || position.size() < dim) {
        return direction;
    }

    double delta = grid_map_->getResolution();
    double pred = grid_map_->atPosition("prediction", pos);
    Vector pos_minor(dim);
    Vector pos_major(dim);
    double pred_minor;
    double pred_major;

    for (int i = 0; i < dim; i++) {
        pos_minor = pos;
        pos_major = pos;
        pos_minor[i] -= delta;
        pos_major[i] += delta;

        if (grid_map_->isInside(pos_minor)) {
            pred_minor = grid_map_->atPosition("prediction", pos_minor);
            if (pred_minor > pred) {
                direction[i] = -1;
            }
        }
        if (grid_map_->isInside(pos_major)) {
            pred_major = grid_map_->atPosition("prediction", pos_major);
            if (pred_major > pred && pred_major > pred_minor) {
                direction[i] = 1;
            }
        }
    }
    return direction;
}

void GaussianProcess2D::confirmSource(int source_id) {
    for (std::shared_ptr<SourceInteractive> &source: sources_) {
        if (source->getId() == source_id) {
            if (source->isConfirmed()) {
                STREAM("Source " << source_id << " unconfirmed.");
                source->setConfirmed(false);
            } else {
                if (!source->wasConfirmed()) {
                    STREAM("Source " << source_id << " confirmed.");
                    GaussianProcess3D::instance().addSourceCandidate(source);
                }
                source->setConfirmed(true);
            }
            break;
        }
    }
}

void GaussianProcess2D::interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK) {
        STREAM_DEBUG("interactiveMarkerCallback : << " << feedback->marker_name);
        confirmSource(std::stoi(feedback->marker_name));
    }
}

void GaussianProcess2D::setUseCircle(bool use_circle) {
    STREAM("GaussianProcess2D set use circle = " << use_circle);
    use_circle_ = use_circle;
}

void GaussianProcess2D::slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg_ptr) {
    slam_map_ = std::make_shared<nav_msgs::OccupancyGrid>(*grid_msg_ptr);
}

void GaussianProcess2D::setMinUpdateTime(int time) {
    STREAM(group_name_ + " set min map update time = " << time);
    this->min_update_time_ = time;
}

void GaussianProcess2D::triggerEvaluation() {
    do_evaluation_ = true;
}
