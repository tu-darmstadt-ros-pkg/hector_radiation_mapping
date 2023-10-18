#include "util/parameters.h"
#include "hector_radiation_mapping/sampleManager.h"
#include "models/gpython/gpython3D.h"
#include "util/dddynamic_reconfigure.h"
#include "models/model_manager.h"


GPython3D::GPython3D() {
    groupName_ = "GPython3D";
    minUpdateTime_ = Parameters::instance().minUpdateTime3d;
    environmentCloudSub_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().nodeHandle_->subscribe(Parameters::instance().environmentCloudTopic,
                                                          Parameters::instance().doseSubSize,
                                                          &GPython3D::environmentCloudCallback, this));
    DDDynamicReconfigure::instance().registerVariable<int>(groupName_ + "_minUpdateTime", minUpdateTime_,
                                                           boost::bind(&GPython3D::setMinUpdateTime, this, _1),
                                                           "min/max",
                                                           0, 5000, groupName_);
    update_ = Parameters::instance().enableOnline3DEvaluation;
    DDDynamicReconfigure::instance().registerVariable<bool>(groupName_ + "_update", update_,
                                                            boost::bind(&GPython3D::setUpdate, this, _1), "on/off",
                                                            false, true, groupName_);
    DDDynamicReconfigure::instance().publish();
    pointCloud_ = std::make_shared<PointCloud3D>("pointCloud3D");
    active_ = false;
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
    updateThread_ = std::thread(&GPython3D::updateLoop, this);
    STREAM_DEBUG("GPython3D activated");
}

void GPython3D::deactivate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (!active_) return;
    this->active_ = false;
    updateThread_.join();
    STREAM_DEBUG("GPython2D deactivated");
}

void GPython3D::updateLoop() {
    Clock clock;
    while (active_ && ros::ok()) {
        clock.tick();
        if (update_ && doEvaluation_) {
            doEvaluation_ = false;
            std::lock_guard<std::mutex> lock1{GPython::instance().getModelMutex()};
            std::lock_guard<std::mutex> lock{pointCloud_->getPointCloudMutex()};
            Matrix samplePositions = getSamplePositions(SampleManager::instance().getLastSamplePos());
            GPython::GPResult gpResult = GPython::instance().evaluate(samplePositions);
            updatePointCloud(gpResult.mean, gpResult.stdDev);
            currentSource_ = getSourcePrediction(samplePositions, gpResult.mean);
        }
        int sleepTime = std::max(0, minUpdateTime_ - (int) clock.tock());
        STREAM_DEBUG("GPython3D sleeping for " << sleepTime << "ms" << " " << minUpdateTime_);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
    }
}

Matrix GPython3D::getSamplePositions(const Vector3d &center) {
    std::lock_guard<std::mutex> lock{environmentCloud_mtx_};
    if (environmentCloud_ == nullptr) {
        STREAM_DEBUG("GPython3D has no EnvironmentCloud");
        return {};
    }
    pointCloud_->generatePointCloudFromEnvironmentCloud(environmentCloud_, environmentCloudTransform_, center);
    return pointCloud_->getMapSamplePositions();
}

void GPython3D::updatePointCloud(const Vector &mean, const Vector &std_dev) {
    pointCloud_->updateMapValues(mean, std_dev);
    pointCloud_->publish();
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
    STREAM_DEBUG(groupName_ + " set min map update time = " << time);
    this->minUpdateTime_ = time;
}

void
GPython3D::environmentCloudCallback(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr &pointCloudMsgPtr) {
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> environmentCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*pointCloudMsgPtr, *environmentCloud);

    // Obtain tf transform from map frame to environment cloud frame
    geometry_msgs::TransformStamped environmentCloudTransform;
    try {
        environmentCloudTransform = SampleManager::instance().getTfBuffer().lookupTransform(
                "world", environmentCloud->header.frame_id, ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    std::lock_guard<std::mutex> lock{environmentCloud_mtx_};
    environmentCloud_ = environmentCloud;
    environmentCloudTransform_ = environmentCloudTransform;
}

void GPython3D::addSourceCandidate(std::shared_ptr<SourceInteractive> &sourceCandidate) {
    // check if sourceCandidate is already in sourceCandidates_
    {
        std::lock_guard<std::mutex> lock{addSource_mtx_};
        for (auto &source: sourceCandidates_) {
            if (sourceCandidate->getId() == source->getId()) {
                return;
            }
        }
        sourceCandidates_.push_back(sourceCandidate);
    }
    GPython::instance().addSamplesWithinRadius(sourceCandidate->getPos(), Parameters::instance().gpLocalRadius, false, true);
}

pcl::PointCloud<PointCloud3D::PointXYZPC> GPython3D::getExportPointCloud() {
    std::lock_guard<std::mutex> lock1{GPython::instance().getModelMutex()};
    sources_.clear();
    pcl::PointCloud<PointCloud3D::PointXYZPC> exportPointCloud;
    int stepSize = 1;

    // get positions of all sourceCandidates
    STREAM_DEBUG("GPython3D: get positions of all sourceCandidates");
    std::vector<Vector3d> sourceCandidatePositions;
    {
        std::lock_guard<std::mutex> lock{addSource_mtx_};
        for (auto &sourceCandidate: sourceCandidates_) {
            if (sourceCandidate->isConfirmed()) {
                sourceCandidatePositions.push_back(sourceCandidate->getPos());
            }
        }
    }

    STREAM_DEBUG("GPython3D: sourceCandidatePositions.size() = " << sourceCandidatePositions.size());
    std::lock_guard<std::mutex> lock{pointCloud_->getPointCloudMutex()};
    for (Vector3d &sourcePositon: sourceCandidatePositions) {
        // Generate PointCloud3D from SourceCandidate
        Matrix samplePositions = getSamplePositions(sourcePositon);
        GPython::GPResult gpResult = GPython::instance().evaluate(samplePositions);
        pointCloud_->updateMapValues(gpResult.mean, gpResult.stdDev);

        // Make source prediction from PointCloud3D
        std::shared_ptr<Source> source = getSourcePrediction(samplePositions, gpResult.mean);
        source->setConfirmed(true);
        sources_.push_back(source);

        // Add Points from PointCloud3D to exportPointCloud
        exportPointCloud.reserve(exportPointCloud.size() + pointCloud_->getPointCloud().size());
        for (PointCloud3D::PointXYZPC &point: pointCloud_->getPointCloud()) {
            exportPointCloud.push_back(point);
        }

        // Update stepSize
        if (pointCloud_->getStepSize() > stepSize) {
            stepSize = ceil(pointCloud_->getStepSize());
        }
    }

    // Get downsampled EnvironmentCloud
    STREAM_DEBUG("GPython3D: Get downsampled EnvironmentCloud");
    {
        std::lock_guard<std::mutex> lock2{environmentCloud_mtx_};
        pointCloud_->generatePointCloudFromEnvironmentCloud(environmentCloud_, environmentCloudTransform_, stepSize);
    }

    // Add points from downsampled EnvironmentCloud to exportPointCloud if they are not in LocalModels
    STREAM_DEBUG("GPython3D: Add points from downsampled EnvironmentCloud to exportPointCloud if they are not in LocalModels");
    double maxDistance2 = pointCloud_->getPointCloudRadius() * pointCloud_->getPointCloudRadius();
    for (PointCloud3D::PointXYZPC &point: pointCloud_->getPointCloud()) {
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
    doEvaluation_ = true;
}
