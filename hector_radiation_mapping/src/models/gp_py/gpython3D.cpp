#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "hector_radiation_mapping/sampleManager.h"
#include "models/model_manager.h"
#include "models/gpython/gpython3D.h"

GPython3D::GPython3D() {
    group_name_ = GPython::instance().getShortModelName();
    std::string prefix = group_name_ + "3D";
    min_update_time_ = Parameters::instance().gp_min_update_time_3d;
    environment_cloud_sub_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe(Parameters::instance().environment_cloud_topic,
                                                               Parameters::instance().dose_sub_size,
                                                               &GPython3D::environmentCloudCallback, this));
    DDDynamicReconfigure::instance().registerVariable<int>(prefix + "_minUpdateTime", min_update_time_,
                                                           boost::bind(&GPython3D::setMinUpdateTime, this, _1),
                                                           "min/max",
                                                           0, 5000, group_name_);
    update_ = Parameters::instance().enable_online_3d_evaluation;
    DDDynamicReconfigure::instance().registerVariable<bool>(prefix + "_update", update_,
                                                            boost::bind(&GPython3D::setUpdate, this, _1), "on/off",
                                                            false, true, group_name_);
    DDDynamicReconfigure::instance().publish();
    point_cloud_ = std::make_shared<PointCloud3D>("pointCloud3D");
    active_ = false;
    do_evaluation_ = false;
    activate();
}

GPython3D &GPython3D::instance() {
    static GPython3D instance;
    return instance;
}

void GPython3D::shutDown() {
    deactivate();
}

void GPython3D::activate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (active_) return;
    this->active_ = true;
    update_thread_ = std::thread(&GPython3D::updateLoop, this);
    STREAM_DEBUG("GPython3D activated");
}

void GPython3D::deactivate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (!active_) return;
    this->active_ = false;
    update_thread_.join();
    STREAM_DEBUG("GPython2D deactivated");
}

void GPython3D::updateLoop() {
    Clock clock;
    while (active_ && ros::ok()) {
        clock.tick();
        if (update_ && do_evaluation_) {
            do_evaluation_ = false;
            std::lock_guard<std::recursive_mutex> lock1{GPython::instance().getModelMutex()};
            std::lock_guard<std::mutex> lock{point_cloud_->getPointCloudMutex()};
            Matrix samplePositions = getSamplePositions(SampleManager::instance().getLastSamplePos());
            GPython::GPResult gpResult = GPython::instance().evaluate(samplePositions);
            updatePointCloud(gpResult.mean, gpResult.std_dev);
            current_source_ = getSourcePrediction(samplePositions, gpResult.mean);
        }
        int sleepTime = std::max(0, min_update_time_ - (int) clock.tock());
        STREAM_DEBUG("GPython3D sleeping for " << sleepTime << "ms" << " " << min_update_time_);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
    }
}

Matrix GPython3D::getSamplePositions(const Vector3d &center) {
    std::lock_guard<std::mutex> lock{environment_cloud_mtx_};
    if (environment_cloud_ == nullptr) {
        STREAM_DEBUG("GPython3D has no EnvironmentCloud");
        return {};
    }
    point_cloud_->generatePointCloudFromEnvironmentCloud(environment_cloud_, environment_cloud_transform_, center);
    return point_cloud_->getMapSamplePositions();
}

void GPython3D::updatePointCloud(const Vector &mean, const Vector &std_dev) {
    point_cloud_->updateMapValues(mean, std_dev);
    point_cloud_->publish();
}

std::shared_ptr<Source> GPython3D::getSourcePrediction(const Matrix &positions, const Vector &predictions) {
    if (positions.rows() != predictions.size()) {
        STREAM_DEBUG("GPython3D: positions.rows() != predictions.size()");
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

void GPython3D::setMinUpdateTime(int time) {
    STREAM_DEBUG(group_name_ + " set min map update time = " << time);
    this->min_update_time_ = time;
}

void
GPython3D::environmentCloudCallback(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr &point_cloud_msg_ptr) {
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> environmentCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*point_cloud_msg_ptr, *environmentCloud);

    // Obtain tf transform from map frame to environment cloud frame
    geometry_msgs::TransformStamped environmentCloudTransform;
    try {
        environmentCloudTransform = SampleManager::instance().getTfBuffer().lookupTransform(
                "world", environmentCloud->header.frame_id, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    std::lock_guard<std::mutex> lock{environment_cloud_mtx_};
    environment_cloud_ = environmentCloud;
    environment_cloud_transform_ = environmentCloudTransform;
}

void GPython3D::addSourceCandidate(std::shared_ptr<SourceInteractive> &source_candidate) {
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
    GPython::instance().addSamples(samples);
}

pcl::PointCloud<PointCloud3D::PointXYZPC> GPython3D::getExportPointCloud() {
    std::lock_guard<std::recursive_mutex> lock1{GPython::instance().getModelMutex()};
    sources_.clear();
    pcl::PointCloud<PointCloud3D::PointXYZPC> exportPointCloud;
    int stepSize = 1;

    // get positions of all sourceCandidates
    STREAM_DEBUG("GPython3D: get positions of all sourceCandidates");
    std::vector<Vector3d> sourceCandidatePositions;
    {
        std::lock_guard<std::mutex> lock{add_source_mtx_};
        for (auto &sourceCandidate: source_candidates_) {
            if (sourceCandidate->isConfirmed()) {
                sourceCandidatePositions.push_back(sourceCandidate->getPos());
            }
        }
    }

    STREAM_DEBUG("GPython3D: sourceCandidatePositions.size() = " << sourceCandidatePositions.size());
    std::lock_guard<std::mutex> lock{point_cloud_->getPointCloudMutex()};
    for (Vector3d &sourcePositon: sourceCandidatePositions) {
        // Generate PointCloud3D from SourceCandidate
        Matrix samplePositions = getSamplePositions(sourcePositon);
        GPython::GPResult gpResult = GPython::instance().evaluate(samplePositions);
        point_cloud_->updateMapValues(gpResult.mean, gpResult.std_dev);

        // Make source prediction from PointCloud3D
        std::shared_ptr<Source> source = getSourcePrediction(samplePositions, gpResult.mean);
        source->setConfirmed(true);
        sources_.push_back(source);

        // Add Points from PointCloud3D to exportPointCloud
        exportPointCloud.reserve(exportPointCloud.size() + point_cloud_->getPointCloud().size());
        for (PointCloud3D::PointXYZPC &point: point_cloud_->getPointCloud()) {
            exportPointCloud.push_back(point);
        }

        // Update stepSize
        if (point_cloud_->getStepSize() > stepSize) {
            stepSize = ceil(point_cloud_->getStepSize());
        }
    }

    // Get downsampled EnvironmentCloud
    STREAM_DEBUG("GPython3D: Get downsampled EnvironmentCloud");
    {
        std::lock_guard<std::mutex> lock2{environment_cloud_mtx_};
        point_cloud_->generatePointCloudFromEnvironmentCloud(environment_cloud_, environment_cloud_transform_, stepSize);
    }

    // Add points from downsampled EnvironmentCloud to exportPointCloud if they are not in LocalModels
    STREAM_DEBUG("GPython3D: Add points from downsampled EnvironmentCloud to exportPointCloud if they are not in LocalModels");
    double maxDistance2 = point_cloud_->getPointCloudRadius() * point_cloud_->getPointCloudRadius();
    for (PointCloud3D::PointXYZPC &point: point_cloud_->getPointCloud()) {
        bool addPoint = true;
        for (Vector3d &sourcePositon: sourceCandidatePositions) {
            Vector3d p(point.x, point.y, point.z);
            if ((p - sourcePositon).squaredNorm() < maxDistance2) {
                addPoint = false;
                break;
            }
        }
        if (addPoint) {
            exportPointCloud.push_back(point);
        }
    }
    STREAM_DEBUG("GPython3D: exportPointCloud.size() = " << exportPointCloud.size() << " stepSize = " << stepSize);
    return exportPointCloud;
}

void GPython3D::setUpdate(bool update) {
    update_ = update;
}

void GPython3D::triggerEvaluation() {
    do_evaluation_ = true;
}
