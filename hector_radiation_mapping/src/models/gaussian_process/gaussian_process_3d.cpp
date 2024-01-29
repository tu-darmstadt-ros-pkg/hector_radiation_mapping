#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "hector_radiation_mapping/sampleManager.h"
#include "models/model_manager.h"
#include "models/gpython/gaussian_process_3d.h"

GaussianProcess3D::GaussianProcess3D() {
    group_name_ = GaussianProcess::instance().getShortModelName();
    std::string prefix = group_name_ + "3D";
    min_update_time_ = Parameters::instance().gp_min_update_time_3d;
    environment_cloud_sub_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe(Parameters::instance().environment_cloud_topic,
                                                               Parameters::instance().dose_sub_size,
                                                               &GaussianProcess3D::environmentCloudCallback, this));
    DDDynamicReconfigure::instance().registerVariable<int>(prefix + "_minUpdateTime", min_update_time_,
                                                           boost::bind(&GaussianProcess3D::setMinUpdateTime, this, _1),
                                                           "min/max",
                                                           0, 5000, group_name_);
    update_ = Parameters::instance().enable_online_3d_evaluation;
    DDDynamicReconfigure::instance().registerVariable<bool>(prefix + "_update", update_,
                                                            boost::bind(&GaussianProcess3D::setUpdate, this, _1), "on/off",
                                                            false, true, group_name_);
    point_cloud_ = std::make_shared<PointCloud3D>("pointCloud3D");
    active_ = false;
    do_evaluation_ = false;
}

GaussianProcess3D &GaussianProcess3D::instance() {
    static GaussianProcess3D instance;
    return instance;
}

void GaussianProcess3D::shutDown() {
    deactivate();
}

void GaussianProcess3D::activate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (active_) return;
    this->active_ = true;
    update_thread_ = std::thread(&GaussianProcess3D::updateLoop, this);
    STREAM_DEBUG("GaussianProcess3D activated");
}

void GaussianProcess3D::deactivate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (!active_) return;
    this->active_ = false;
    update_thread_.join();
    STREAM_DEBUG("GaussianProcess2D deactivated");
}

void GaussianProcess3D::updateLoop() {
    Clock clock;
    while (active_ && ros::ok()) {
        clock.tick();
        if (update_ && do_evaluation_) {
            do_evaluation_ = false;
            std::lock_guard<std::recursive_mutex> lock1{GaussianProcess::instance().getModelMutex()};
            std::lock_guard<std::mutex> lock{point_cloud_->getPointCloudMutex()};
            Matrix samplePositions = getSamplePositions(SampleManager::instance().getLastSamplePos());
            GaussianProcess::GPResult gpResult = GaussianProcess::instance().evaluate(samplePositions);
            updatePointCloud(gpResult.mean, gpResult.std_dev);
            current_source_ = getSourcePrediction(samplePositions, gpResult.mean);
        }
        int sleepTime = std::max(0, min_update_time_ - (int) clock.tock());
        STREAM_DEBUG("GaussianProcess3D sleeping for " << sleepTime << "ms" << " " << min_update_time_);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
    }
}

Matrix GaussianProcess3D::getSamplePositions(const Vector3d &center) {
    std::lock_guard<std::mutex> lock{environment_cloud_mtx_};
    if (environment_cloud_ == nullptr) {
        STREAM_DEBUG("GaussianProcess3D has no EnvironmentCloud");
        return {};
    }
    point_cloud_->generatePointCloudFromEnvironmentCloud(environment_cloud_, environment_cloud_transform_, center);
    return point_cloud_->getMapSamplePositions();
}

void GaussianProcess3D::updatePointCloud(const Vector &mean, const Vector &std_dev) {
    point_cloud_->updateMapValues(mean, std_dev);
    point_cloud_->publish();
}

std::shared_ptr<Source> GaussianProcess3D::getSourcePrediction(const Matrix &positions, const Vector &predictions) {
    if (positions.rows() != predictions.size()) {
        STREAM_DEBUG("GaussianProcess3D: positions.rows() != predictions.size()");
        return nullptr;
    }
    double max_pred = 0.0;
    Eigen::Vector3d max_pos;
    for (int i = 0; i < positions.rows(); i++) {
        if (predictions(i) > max_pred) {
            max_pred = predictions(i);
            max_pos = positions.row(i);
        }
    }
    return std::make_shared<Source>(max_pos, max_pred, max_pred);
}

void GaussianProcess3D::setMinUpdateTime(int time) {
    STREAM_DEBUG(group_name_ + " set min map update time = " << time);
    this->min_update_time_ = time;
}

void
GaussianProcess3D::environmentCloudCallback(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr &point_cloud_msg_ptr) {
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> environment_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*point_cloud_msg_ptr, *environment_cloud);

    // Obtain tf transform from map frame to environment cloud frame
    geometry_msgs::TransformStamped environment_cloud_transform;
    try {
        environment_cloud_transform = SampleManager::instance().getTfBuffer().lookupTransform(
                "world", environment_cloud->header.frame_id, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    std::lock_guard<std::mutex> lock{environment_cloud_mtx_};
    environment_cloud_ = environment_cloud;
    environment_cloud_transform_ = environment_cloud_transform;
}

void GaussianProcess3D::addSourceCandidate(std::shared_ptr<SourceInteractive> &source_candidate) {
    // check if source_candidate is already in source_candidates_
    {
        std::lock_guard<std::mutex> lock{add_source_mtx_};
        for (auto &source: source_candidates_) {
            if (source_candidate->getId() == source->getId()) {
                return;
            }
        }
        source_candidates_.push_back(source_candidate);
    }

    std::vector<Sample> samples = SampleManager::instance().getSamplesWithinRadius(source_candidate->getPos(), Parameters::instance().gp_local_radius);
    GaussianProcess::instance().addSamples(samples);
}

pcl::PointCloud<PointCloud3D::PointXYZPC> GaussianProcess3D::getExportPointCloud() {
    std::lock_guard<std::recursive_mutex> lock1{GaussianProcess::instance().getModelMutex()};
    sources_.clear();
    pcl::PointCloud<PointCloud3D::PointXYZPC> export_point_cloud;
    int step_size = 1;

    // get positions of all sourceCandidates
    STREAM_DEBUG("GaussianProcess3D: get positions of all sourceCandidates");
    std::vector<Vector3d> source_candidate_positions;
    {
        std::lock_guard<std::mutex> lock{add_source_mtx_};
        for (auto &source_candidate: source_candidates_) {
            if (source_candidate->isConfirmed()) {
                source_candidate_positions.push_back(source_candidate->getPos());
            }
        }
    }

    STREAM_DEBUG("GaussianProcess3D: source_candidate_positions.size() = " << source_candidate_positions.size());
    std::lock_guard<std::mutex> lock{point_cloud_->getPointCloudMutex()};
    for (Vector3d &sourcePositon: source_candidate_positions) {
        // Generate PointCloud3D from SourceCandidate
        Matrix sample_positions = getSamplePositions(sourcePositon);
        GaussianProcess::GPResult gp_result = GaussianProcess::instance().evaluate(sample_positions);
        point_cloud_->updateMapValues(gp_result.mean, gp_result.std_dev);

        // Make source prediction from PointCloud3D
        std::shared_ptr<Source> source = getSourcePrediction(sample_positions, gp_result.mean);
        source->setConfirmed(true);
        sources_.push_back(source);

        // Add Points from PointCloud3D to export_point_cloud
        export_point_cloud.reserve(export_point_cloud.size() + point_cloud_->getPointCloud().size());
        for (PointCloud3D::PointXYZPC &point: point_cloud_->getPointCloud()) {
            export_point_cloud.push_back(point);
        }

        // Update step_size
        if (point_cloud_->getStepSize() > step_size) {
            step_size = ceil(point_cloud_->getStepSize());
        }
    }

    // Get downsampled EnvironmentCloud
    STREAM_DEBUG("GaussianProcess3D: Get downsampled EnvironmentCloud");
    {
        std::lock_guard<std::mutex> lock2{environment_cloud_mtx_};
        point_cloud_->generatePointCloudFromEnvironmentCloud(environment_cloud_, environment_cloud_transform_, step_size);
    }

    // Add points from downsampled EnvironmentCloud to export_point_cloud if they are not in LocalModels
    STREAM_DEBUG("GaussianProcess3D: Add points from downsampled EnvironmentCloud to export_point_cloud if they are not in LocalModels");
    double max_distance2 = point_cloud_->getPointCloudRadius() * point_cloud_->getPointCloudRadius();
    for (PointCloud3D::PointXYZPC &point: point_cloud_->getPointCloud()) {
        bool add_point = true;
        for (Vector3d &source_positon: source_candidate_positions) {
            Vector3d p(point.x, point.y, point.z);
            if ((p - source_positon).squaredNorm() < max_distance2) {
                add_point = false;
                break;
            }
        }
        if (add_point) {
            export_point_cloud.push_back(point);
        }
    }
    STREAM_DEBUG("GaussianProcess3D: export_point_cloud.size() = " << export_point_cloud.size() << " step_size = " << step_size);
    return export_point_cloud;
}

void GaussianProcess3D::setUpdate(bool update) {
    update_ = update;
}

void GaussianProcess3D::triggerEvaluation() {
    do_evaluation_ = true;
}
