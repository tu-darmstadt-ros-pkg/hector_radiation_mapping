#include "models/gpython/gpython2D.h"
#include "models/gpython/gpython3D.h"
#include "models/model_exporter.h"
#include "hector_radiation_mapping/sampleManager.h"
#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "util/util.h"
#include "util/clock_cpu.h"


GPython2D::GPython2D() {
    int id = 0;
    std::string mapName = "grid_map_" + std::to_string(id);
    std::string modelName = "GPGlobal_" + std::to_string(id);
    layerNameMean_ = "prediction";
    layerNameStdDev_ = "stdDev";
    gridMap_ = std::make_shared<GridMap>(mapName, Parameters::instance().gridMapResolution);
    gridMap_->addLayer(layerNameMean_);
    gridMap_->addLayer(layerNameStdDev_);
    useCircle_ = true;
    doEvaluation_ = false;
    groupName_ = "GPython2D";
    minUpdateTime_ = Parameters::instance().minUpdateTime2d;
    DDDynamicReconfigure::instance().registerVariable<bool>(groupName_ + "_useCircleEvaluation", useCircle_,
                                                            boost::bind(&GPython2D::setUseCircle, this, _1), "on/off",
                                                            false, true, groupName_);
    DDDynamicReconfigure::instance().registerVariable<int>(groupName_ + "_minUpdateTime", minUpdateTime_,
                                                           boost::bind(&GPython2D::setMinUpdateTime, this, _1),
                                                           "min/max",
                                                           0, 5000, groupName_);
    DDDynamicReconfigure::instance().publish();

    slamMapSubscriber_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().nodeHandle_->subscribe(Parameters::instance().environmentMapTopic,
                                                          Parameters::instance().doseSubSize,
                                                          &GPython2D::slamMapCallback, this));

    active_ = false;
    activate();
}

GPython2D &GPython2D::instance() {
    static GPython2D instance;
    return instance;
}

void GPython2D::shutDown() {
    deactivate();
}

void GPython2D::activate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (active_) return;
    this->active_ = true;
    updateThread_ = std::thread(&GPython2D::updateLoop, this);
    STREAM_DEBUG("GPython2D activated");
}

void GPython2D::deactivate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (!active_) return;
    this->active_ = false;
    updateThread_.join();
    STREAM_DEBUG("GPython2D deactivated");
}

void GPython2D::updateLoop() {
    Clock clock;
    std::vector<double> evaluationTimes;
    std::vector<double> sourcePredTimes;
    std::vector<double> evaluationSizes;
    while (active_ && ros::ok()) {
        clock.tick();
        if (doEvaluation_) {
            doEvaluation_ = false;
            bool useCircle = useCircle_;
            Vector2d center = SampleManager::instance().getLastSamplePos().topRows(2);
            std::shared_ptr<nav_msgs::OccupancyGrid> slamMap = slamMap_;
            {
                std::lock_guard<std::mutex> lock1{GPython::instance().getModelMutex()};
                std::lock_guard<std::mutex> lock{gridMap_->getGridMapMutex()};
                Matrix samplePositions = getSamplePositions(slamMap, useCircle, center);
                GPython::GPResult gpResult = GPython::instance().evaluate(samplePositions);
                updateMap(gpResult.mean, gpResult.stdDev, useCircle, center);
                evaluationTimes.push_back((double) clock.tock());
                clock.tick();
                updateSourcePrediction(samplePositions, gpResult.mean);
                sourcePredTimes.push_back((double) clock.tock());
            }
            evaluationSizes.push_back((double) GPython::instance().getSampleIds2d().size());
        }
        int sleepTime = std::max(0, minUpdateTime_ - (int) clock.tock());
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
    }
    //std::string exportPath = Util::getExportPath("runtime");
    //Util::exportVectorToTxtFile(evaluationTimes, exportPath, "evaluationTimes", Util::TxtExportType::NEW, true);
    //Util::exportVectorToTxtFile(sourcePredTimes, exportPath, "sourcePredTimes", Util::TxtExportType::NEW, true);
    //Util::exportVectorToTxtFile(evaluationSizes, exportPath, "evaluationSizes", Util::TxtExportType::NEW, true);
}

Matrix GPython2D::getSamplePositions(const std::shared_ptr<nav_msgs::OccupancyGrid> &slamMap, bool useCircle,
                                     const Vector2d &center) {
    double radius = Parameters::instance().circleRadius;
    if (slamMap != nullptr) {
        gridMap_->updateGridDimensionWithSlamMap(slamMap);
    }
    return useCircle ? gridMap_->getCircleSamplePositions(center, radius)
                     : gridMap_->getMapSamplePositions();
}

void GPython2D::updateMap(const Vector &mean, const Vector &std_dev, bool useCircle, const Vector2d &center) {
    double radius = Parameters::instance().circleRadius;
    if (useCircle) {
        gridMap_->updateLayer(layerNameMean_, mean, center, radius);
        gridMap_->updateLayer(layerNameStdDev_, std_dev, center, radius);
    } else {
        gridMap_->updateLayer(layerNameMean_, mean);
        gridMap_->updateLayer(layerNameStdDev_, std_dev);
    }
    gridMap_->publish();
}

std::pair<grid_map::GridMap, std::shared_ptr<nav_msgs::OccupancyGrid>> GPython2D::getExportGridMap() {
    std::lock_guard<std::mutex> lock{gridMap_->getGridMapMutex()};
    std::lock_guard<std::mutex> lock1{GPython::instance().getModelMutex()};
    std::shared_ptr<nav_msgs::OccupancyGrid> slamMap = slamMap_;
    Matrix samplePositions = getSamplePositions(slamMap);
    GPython::GPResult gpResult = GPython::instance().evaluate(samplePositions);
    updateMap(gpResult.mean, gpResult.stdDev);
    return {gridMap_->getGridMap(), slamMap};
}

void GPython2D::updateSourcePrediction(const Matrix &positions, const Vector &predictions) {
    int maxOptimizerSteps = 1000;
    double minSourceDistance = 2 * gridMap_->getResolution();
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

    // Generate start positions (All points in trajectory with at least minDist to each other)
    Eigen::Matrix3Xd startPositions;
    Vector3d lastPos;
    Vector3d pos3D;
    lastPos << -DBL_MAX, -DBL_MAX, -DBL_MAX;
    double mean = 0;
    double minDist = 1;
    double dist;

    std::vector<Sample> samples = SampleManager::instance().getSamples();
    for (const Sample &sample: samples) {
        pos3D = sample.position_;
        if (!gridMap_->isInside(pos3D.topRows(dim))) {
            continue;
        }
        dist = (pos3D - lastPos).norm();
        if (dist > minDist) {
            startPositions.conservativeResize(3, startPositions.cols() + 1);
            startPositions.col(startPositions.cols() - 1) = pos3D;
            lastPos = pos3D;
            mean += gridMap_->atPosition("prediction", pos3D.topRows(dim));
        }
    }
    mean = mean / startPositions.cols();
    STREAM_DEBUG("GP2D Start positions: " << startPositions.cols() << " Mean: " << mean);

    // Gradient ascent for each start position
    int steps;
    double dosage = 0.0;
    bool tooClose;

    for (int i = 0; i < startPositions.cols(); i++) {
        pos3D = startPositions.col(i);
        dir = computeGradientStep(pos3D);

        steps = 0;
        while (!dir.isApprox(dirZero) && steps < maxOptimizerSteps) {
            dir = computeGradientStep(pos3D);
            pos3D.topRows(dim) += dir * gridMap_->getResolution();
            steps += 1;
            dosage = gridMap_->atPosition("prediction", pos3D.topRows(dim));
        }

        tooClose = false;
        for (std::shared_ptr<SourceInteractive> &source: sources_) {
            if ((source->getPos() - pos3D).topRows(dim).squaredNorm() < minSourceDistance * minSourceDistance) {
                tooClose = true;
                break;
            }
        }

        if (!tooClose && dosage > Parameters::instance().meanFactor * mean) {
            if (dosage < 0.1) {
                continue;
            }
            STREAM_DEBUG("GP2D Adding source: ");
            Vector3d pos = Vector3d(pos3D.x(), pos3D.y(), 0.0);
            std::shared_ptr<SourceInteractive> source = std::make_shared<SourceInteractive>(pos, dosage, dosage,
                                                                                            boost::bind(
                                                                                                    &GPython2D::interactiveMarkerCallback,
                                                                                                    this, _1));
            sources_.push_back(source);
            STREAM_DEBUG("GP2D Adding source: " + std::to_string(source->getStrength()));
        }
    }
    STREAM_DEBUG("Done with sources");
}

Vector GPython2D::computeGradientStep(const Vector &position) {
    int dim = 2;
    Vector direction = Vector::Zero(dim);
    Vector pos = position.topRows(dim);

    if (!gridMap_->isInside(pos) || position.size() < dim) {
        return direction;
    }

    double delta = gridMap_->getResolution();
    double pred = gridMap_->atPosition("prediction", pos);
    Vector posMinor(dim);
    Vector posMajor(dim);
    double predMinor;
    double predMajor;

    for (int i = 0; i < dim; i++) {
        posMinor = pos;
        posMajor = pos;
        posMinor[i] -= delta;
        posMajor[i] += delta;

        if (gridMap_->isInside(posMinor)) {
            predMinor = gridMap_->atPosition("prediction", posMinor);
            if (predMinor > pred) {
                direction[i] = -1;
            }
        }
        if (gridMap_->isInside(posMajor)) {
            predMajor = gridMap_->atPosition("prediction", posMajor);
            if (predMajor > pred && predMajor > predMinor) {
                direction[i] = 1;
            }
        }
    }
    return direction;
}

void GPython2D::confirmSource(int sourceId) {
    for (std::shared_ptr<SourceInteractive> &source: sources_) {
        if (source->getId() == sourceId) {
            if (source->isConfirmed()) {
                STREAM("Source " << sourceId << " unconfirmed.");
                source->setConfirmed(false);
            } else {
                if (!source->wasConfirmed()) {
                    STREAM("Source " << sourceId << " confirmed.");
                    GPython3D::instance().addSourceCandidate(source);
                }
                source->setConfirmed(true);
            }
            break;
        }
    }
}

void GPython2D::interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK) {
        STREAM_DEBUG("interactiveMarkerCallback : << " << feedback->marker_name);
        confirmSource(std::stoi(feedback->marker_name));
    }
}

void GPython2D::setUseCircle(bool useCircle) {
    STREAM("GPython2D set use circle = " << useCircle);
    useCircle_ = useCircle;
}

void GPython2D::slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &gridMsgPtr) {
    slamMap_ = std::make_shared<nav_msgs::OccupancyGrid>(*gridMsgPtr);
}

void GPython2D::setMinUpdateTime(int time) {
    STREAM(groupName_ + " set min map update time = " << time);
    this->minUpdateTime_ = time;
}

void GPython2D::triggerEvaluation() {
    doEvaluation_ = true;
}
