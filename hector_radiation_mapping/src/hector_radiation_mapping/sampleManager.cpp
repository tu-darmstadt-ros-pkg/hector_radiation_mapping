#include "hector_radiation_mapping/sampleManager.h"
#include "hector_radiation_mapping/source.h"
#include "hector_radiation_mapping/sample.h"
#include "util/parameters.h"
#include "models/model_manager.h"
#include "models/gpython/gpython.h"

SampleManager::SampleManager() {
    // Fix topic name if necessary
    if (Parameters::instance().subscribeTopic.at(0) != '/') {
        Parameters::instance().subscribeTopic = "/" + Parameters::instance().subscribeTopic;
    }

    // Init message parser
    babelFish_ = std::make_shared<ros_babel_fish::BabelFish>();
    universalSub_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().nodeHandle_->subscribe<ros_babel_fish::BabelFishMessage>(
                    Parameters::instance().subscribeTopic,
                    Parameters::instance().doseSubSize,
                    &SampleManager::doseCallbackFish, this
            ));
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);

    // initialize vars
    backgroundRadiationCps_ = 0.0;
    backgroundRadiationDoseRate_ = 0.0;
    trajectoryMarker_ = std::make_shared<TrajectoryMarker>();
}

SampleManager &SampleManager::instance() {
    static SampleManager instance;
    return instance;
}

void SampleManager::doseCallbackFish(const ros_babel_fish::BabelFishMessage::ConstPtr &msg) {
    std::string keyRate = Parameters::instance().messageKey_rate;
    std::string keyCps = Parameters::instance().messageKey_cps;
    std::string keyFrameId = Parameters::instance().messageKey_frameId;
    ros_babel_fish::TranslatedMessage::Ptr translated = babelFish_->translateMessage(msg);
    auto &compound = translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    double doseRate = compound[keyRate].value<float>();
    double cps = -1;
    std::string frameId = compound["header"][keyFrameId].value<std::string>();

    try {
        cps = compound[keyCps].value<float>();
    } catch (...) {
        cps = doseRate * mikroSievertPerHourToCps_;
    }

    try {
        geometry_msgs::Transform trans = tfBuffer_.lookupTransform("world", frameId, ros::Time(0),
                                                                   ros::Duration(1)).transform;
        Eigen::Vector3d pos(trans.translation.x, trans.translation.y, trans.translation.z);

        updateTrajectory(Parameters::instance().useDoseRate ? doseRate : cps, pos);

        static bool calculated_background_radiation = false;
        if (!calculated_background_radiation) {
            static std::vector<double> cpsVec;
            static std::vector<double> doseRateVec;
            static std::vector<Eigen::Vector3d> posVec;
            static std::vector<ros::Time> timeVec;
            cpsVec.push_back(cps);
            doseRateVec.push_back(doseRate);
            posVec.push_back(pos);
            timeVec.push_back(ros::Time::now());
            backgroundRadiationCps_ += cps / 10.0;
            backgroundRadiationDoseRate_ += doseRate / 10.0;

            if (cpsVec.size() >= 10) {
                for (int i = 0; i < cpsVec.size(); i++) {
                    processSampleData(posVec[i], cpsVec[i], doseRateVec[i], timeVec[i]);
                }
                cpsVec.clear();
                doseRateVec.clear();
                posVec.clear();
                timeVec.clear();
                calculated_background_radiation = true;
            }
        } else {
            processSampleData(pos, cps, doseRate, ros::Time::now());
        }

        //double time = sample.time_.toSec() - Parameters::instance().startTime_;
        //std::string exportPath = Util::getExportPath("samples");
        //Util::appendDoubleToTxtFile(cps, exportPath, "cps");
        //Util::appendDoubleToTxtFile(doseRate, exportPath, "doseRate");
        //Util::appendDoubleToTxtFile(time, exportPath, "time");
    } catch (tf2::TransformException &exception) {
        ROS_WARN_STREAM(exception.what());
    }
}


void SampleManager::processSampleData(Vector3d pos, double cps, double doseRate, ros::Time time) {
    /// Get background radiation from average of first few samples
    double adjCps = fmax(cps - backgroundRadiationCps_, 0.0);
    double adjDoseRate = fmax(doseRate - backgroundRadiationDoseRate_, 0.0);
    Sample sample(pos, adjCps, adjDoseRate, time);

    if(!Parameters::instance().enableSpatialSampleFiltering){
        double minDist2 = 0.2 * 0.2;
        if (samples_.empty() || (sample.position_ - getLastSamplePos()).squaredNorm() > minDist2 || sample.doseRate_ > 20.0) {
            addSample(sample);
        }
        return;
    }

    const double distCutoff = 0.005;
    const double distCutoff2 = distCutoff * distCutoff;

    if (sampleQueue_.empty()) {
        sampleQueue_.push_back(sample);
    }
    else {
        /// if position has changed considerably, add new sample and the mean of the old ones that are left in the Queue
        if((sample.position_ - sampleQueue_.back().position_).squaredNorm() > distCutoff2) {
            Sample meanSample = getMeanSample(sampleQueue_, false);
            for(int i = 0; i < ceil((double)sampleQueue_.size()/ 5.0); i++){
                addSample(meanSample);
            }
            addSample(sample);
            sampleQueue_.clear();
        }
        else {
            sampleQueue_.push_back(sample);

            /// if position has not changed much, add mean of all Samples in the Queue if largest distance exceeds threshold
            double maxDist, distSqr = 0.0;
            for(int i = 0; i < sampleQueue_.size() - 1; ++i) {
                for(int j = i + 1; j < sampleQueue_.size(); j++){
                    distSqr = (sampleQueue_[i].position_ - sampleQueue_[j].position_).squaredNorm();
                    if(distSqr > maxDist){
                        maxDist = distSqr;
                    }
                }
            }
            if(maxDist > distCutoff2){
                Sample meanSample = getMeanSample(sampleQueue_, true);
                for(int i = 0; i < ceil((double)sampleQueue_.size()/ 5.0); i++){
                    addSample(meanSample);
                }
                sampleQueue_.clear();
            }
        }
    }
}

void SampleManager::addSample(Sample &sample) {
    samples_.push_back(sample);

    double max = 0.0;
    for (Sample &sample: samples_) {
        if (max < sample.doseRate_) {
            max = sample.doseRate_;
        }
    }
    GPython::SampleGP sampleGP(sample, true, true);
    GPython::instance().addSample(sampleGP);
}

void SampleManager::updateTrajectory(const double &value, const Eigen::Vector3d &position) {
    trajectoryMarker_->addPoint(value, position);
}

Eigen::Vector3d SampleManager::getLastSamplePos() {
    if (samples_.empty()) {
        return {-DBL_MAX, -DBL_MAX, -DBL_MAX};
    }
    return samples_.back().position_;
}

std::vector<Sample> SampleManager::getSamplesWithinRadius(const Eigen::Vector3d &position, double radius) {
    std::vector<Sample> inSphere;
    for (const Sample &sample: samples_) {
        if ((position - sample.position_).squaredNorm() < radius * radius) {
            inSphere.push_back(sample);
        }
    }
    return inSphere;
}

Sample SampleManager::getNearestSample(const Eigen::Vector3d &position) {
    double minDist = DBL_MAX;
    Sample nearestSample;
    for (const Sample &sample: samples_) {
        double dist = (position - sample.position_).squaredNorm();
        if (dist < minDist) {
            minDist = dist;
            nearestSample = sample;
        }
    }
    return nearestSample;
}

Sample SampleManager::getMeanSample(const std::vector<Sample> &samples, bool weighted) {
    Eigen::Vector3d meanPos(0.0, 0.0, 0.0);
    double meanCps = 0.0;
    double meanDoseRate = 0.0;
    double weight = 1.0;
    double totalWeight = 0.0;

    for (const Sample &sample: samples) {
        meanPos = meanPos + weight * sample.position_;
        meanCps = meanCps + weight * sample.cps_;
        meanDoseRate = meanDoseRate + weight * sample.doseRate_;
        totalWeight += weight;
        //weight += 0.5;
    }

    meanPos /= totalWeight;
    meanCps /= totalWeight;
    meanDoseRate /= totalWeight;

    Sample combinedSample(meanPos, meanCps, meanDoseRate, ros::Time::now());
    return combinedSample;
}

std::vector<Sample> SampleManager::getSamplesNotInIdVector(const std::vector<Sample> &samples,
                                                           const std::vector<int> &ids) {
    std::vector<Sample> samplesNotInIdVector;
    for (const Sample &sample: samples) {
        if (std::find(ids.begin(), ids.end(), sample.id_) == ids.end()) {
            samplesNotInIdVector.push_back(sample);
        }
    }
    return samplesNotInIdVector;
}

std::vector<Sample> SampleManager::getSamples() {
    return samples_;
}

tf2_ros::Buffer &SampleManager::getTfBuffer() {
    return tfBuffer_;
}
