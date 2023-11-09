#include <std_msgs/Float64MultiArray.h>
#include <eigen_conversions/eigen_msg.h>

#include "pch.h"
#include "models/gpython/gpython.h"
#include "models/gpython/gpython2D.h"
#include "models/gpython/gpython3D.h"
#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "util/util.h"
#include "util/clock_cpu.h"
#include "hector_radiation_mapping_msgs/GPEvaluationService.h"
#include "hector_radiation_mapping_msgs/GPEvaluationServiceRequest.h"
#include "hector_radiation_mapping_msgs/GPEvaluationServiceResponse.h"
#include "hector_radiation_mapping_msgs/AddSamplesService.h"
#include "hector_radiation_mapping_msgs/AddSamplesServiceRequest.h"
#include "hector_radiation_mapping_msgs/AddSamplesServiceResponse.h"
#include "hector_radiation_mapping_msgs/Samples.h"
#include "hector_radiation_mapping_msgs/Sample.h"
#include "hector_radiation_mapping/sampleManager.h"

GPython::GPython() {
    // Define ROS service clients
    evaluationServiceClient_ = Parameters::instance().nodeHandle_->serviceClient<hector_radiation_mapping_msgs::GPEvaluationService>("/hector_radiation_mapping/gp_evaluation");
    sampleServiceClient_ = Parameters::instance().nodeHandle_->serviceClient<hector_radiation_mapping_msgs::AddSamplesService>("/hector_radiation_mapping/gp_samples");

    // Create dynamic reconfigure parameters
    param1_ptr = new double[1];
    param2_ptr = new double[1];
    param3_ptr = new double[1];
    *param1_ptr = 1.0;
    *param2_ptr = 1.0;
    *param3_ptr = 10.0;
    groupName_ = "gpython";
    DDDynamicReconfigure::instance().registerVariable<double>(groupName_ + "_kernel_lengthscale", param1_ptr, boost::bind(&GPython::paramCallback, this), "param1", 0.0, 4.0, groupName_);
    DDDynamicReconfigure::instance().registerVariable<double>(groupName_ + "_outputscale", param2_ptr, boost::bind(&GPython::paramCallback, this), "param2", 0.0, 20.0, groupName_);
    DDDynamicReconfigure::instance().registerVariable<double>(groupName_ + "_likelihood_noise", param3_ptr, boost::bind(&GPython::paramCallback, this), "param3", 0.0, 20.0, groupName_);
    DDDynamicReconfigure::instance().publish();
    STREAM_DEBUG("GPython params: " << *param1_ptr << " " << *param2_ptr << " " << *param3_ptr);

    // Initialize variables
    active_ = false;
    paramUpdate_ = false;
    modelSize_ = 0;
    activate();
}

GPython &GPython::instance() {
    static GPython instance;
    return instance;
}

void GPython::shutDown() {
    deactivate();
}

bool GPython::isActive() const {
    return active_;
}

void GPython::activate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (active_) return;
    this->active_ = true;
    updateThread_ = std::thread(&GPython::updateLoop, this);
    STREAM_DEBUG("GPython activated");
}

void GPython::deactivate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (!active_) return;
    this->active_ = false;
    waitCondition_.notify_one();
    updateThread_.join();
    STREAM_DEBUG("GPython deactivated");
}

void GPython::addSample(SampleGP &sample) {
    std::lock_guard<std::mutex> lock{sampleQueue_mtx_};
    samplesQueue_.push_back(sample);
    waitCondition_.notify_one();
    modelSize_++;
    // Position with 2 decimals
    double pos_x = round(sample.sample.position_[0] * 100) / 100;
    double pos_y = round(sample.sample.position_[1] * 100) / 100;
    double pos_z = round(sample.sample.position_[2] * 100) / 100;

    STREAM("Added sample #" << modelSize_ << " with dose rate " << sample.sample.doseRate_ << " " << Parameters::instance().radiationUnit 
        << " at [" << pos_x << ", " << pos_y << ", " << pos_z << "].\n");

}

void GPython::addSamples(std::vector<SampleGP> &samples) {
    std::lock_guard<std::mutex> lock{sampleQueue_mtx_};
    samplesQueue_.insert(samplesQueue_.end(), samples.begin(), samples.end());
    waitCondition_.notify_one();
}

void GPython::updateLoop() {
    Clock clock;
    std::vector<SampleGP> samplesQueueCopy;
    while (active_ && ros::ok()) {
        {
            std::unique_lock<std::mutex> lock{sampleQueue_mtx_};
            waitCondition_.wait(lock, [this]() { return (!samplesQueue_.empty()) || !active_ || paramUpdate_;});
            if (!active_) break;
            paramUpdate_ = false;
            GPython2D::instance().triggerEvaluation();
            GPython3D::instance().triggerEvaluation();

            // create copy of samples
            samplesQueueCopy = samplesQueue_;
            samplesQueue_.clear();
        }

        {
            std::lock_guard<std::mutex> lock{model_mtx_};
            clock.tick();
            addSamplesToModel(samplesQueueCopy);
            updateTimes_.push_back((double) clock.tock());
        }
        updateSizes_.push_back((double) sampleIds2d_.size());
    }
    //std::string exportPath = Util::getExportPath("runtime");
    //Util::exportVectorToTxtFile(updateTimes_, exportPath, "updateTimes_", Util::TxtExportType::NEW, true);
    //Util::exportVectorToTxtFile(updateSizes_, exportPath, "updateSizes_", Util::TxtExportType::NEW, true);
}

Vector GPython::varianceToStdDeviation(Vector &variance) {
    return variance.array().sqrt();
}

GPython::GPResult GPython::evaluate(Matrix &positions) {
    // check if positions are 2D or 3D
    if (positions.cols() != 2 && positions.cols() != 3) {
        STREAM_ERROR("GPython: evaluate() positions must be 2D or 3D");
        return {{}, {}};
    }

    hector_radiation_mapping_msgs::GPEvaluationServiceRequest req;
    hector_radiation_mapping_msgs::GPEvaluationServiceResponse res;
    std_msgs::Float64MultiArray positions_msg;
    tf::matrixEigenToMsg(positions, positions_msg);
    req.positions = positions_msg;

    //{
    //    std::lock_guard<std::mutex> lock{model_mtx_};
        evaluationServiceClient_.call(req, res);
    //}

    std_msgs::Float64MultiArray mean = res.mean;
    std_msgs::Float64MultiArray variance = res.variance;

    // convert mean and variance to vector
    Vector mean_vec = Vector::Zero(positions.rows());
    Vector variance_vec = Vector::Zero(positions.rows());

    for (size_t i = 0; i < mean.data.size(); i++) {
        mean_vec[i] = std::max(0.0, mean.data[i]);
        variance_vec[i] = variance.data[i];
    }

    Vector std_dev_vec = varianceToStdDeviation(variance_vec);
    return {mean_vec, variance_vec};
}

void GPython::addSamplesToModel(const std::vector<SampleGP> &samples) {
    hector_radiation_mapping_msgs::Samples samples_msg;
    for (const SampleGP &sample: samples) {
        // check if sample is already in list
        Sample s = sample.sample;
        bool add2d = sample.for2d;
        bool add3d = sample.for3d;
        if (add2d && isSampleIn2DModel(s)) {
            add2d = false;
        }
        if (add3d && isSampleIn3DModel(s)) {
            add3d = false;
        }
        if (!add2d && !add3d) continue;

        // add sample id to id list
        if (add2d) sampleIds2d_.push_back(s.id_);
        if (add3d) sampleIds3d_.push_back(s.id_);

        // create sample message
        hector_radiation_mapping_msgs::Sample sample_msg;
        sample_msg.cps = s.cps_;
        sample_msg.doseRate = s.doseRate_;
        sample_msg.id = s.id_;
        sample_msg.header.stamp = ros::Time::now();
        sample_msg.position.resize(3);
        sample_msg.position[0] = s.position_[0];
        sample_msg.position[1] = s.position_[1];
        sample_msg.position[2] = s.position_[2];
        sample_msg.for2d = add2d;
        sample_msg.for3d = add3d;
        samples_msg.samples.push_back(sample_msg);
    }

    hector_radiation_mapping_msgs::AddSamplesServiceRequest req;
    hector_radiation_mapping_msgs::AddSamplesServiceResponse res;
    req.samples_msgs = samples_msg;
    req.params.resize(3);
    req.params[0] = *param1_ptr;
    req.params[1] = *param2_ptr;
    req.params[2] = *param3_ptr;
    STREAM_DEBUG("Params: " << *param1_ptr << " " << *param2_ptr << " " << *param3_ptr);
    //{
    //    std::lock_guard<std::mutex> lock{model_mtx_};
        sampleServiceClient_.call(req, res);
    //}
}

bool GPython::isSampleIn2DModel(const Sample &sample){
    return std::find(sampleIds2d_.begin(), sampleIds2d_.end(), sample.id_) != sampleIds2d_.end();
}

bool GPython::isSampleIn3DModel(const Sample &sample){
    return std::find(sampleIds3d_.begin(), sampleIds3d_.end(), sample.id_) != sampleIds3d_.end();
}

void GPython::addSamplesWithinRadius(const Vector3d& position, double radius, bool for2d, bool for3d) {
    std::vector<SampleGP> samples;
    double radius2 = radius * radius;
    for (const Sample &sample: SampleManager::instance().getSamples()) {
        if ((position - sample.position_).squaredNorm() < radius2) {
            samples.emplace_back(sample, for2d, for3d);
        }
    }
    addSamples(samples);
}

void GPython::paramCallback() {
    STREAM_DEBUG("GPython params changed " << *param1_ptr << " " << *param2_ptr << " " << *param3_ptr);
    paramUpdate_ = true;
    waitCondition_.notify_one();
}
