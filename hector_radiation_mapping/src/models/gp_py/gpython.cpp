#include <std_msgs/Float64MultiArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <cstdlib>

#include "models/gpython/gpython.h"
#include "models/gpython/gpython2D.h"
#include "models/gpython/gpython3D.h"
#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "util/util.h"
#include "hector_radiation_mapping/sampleManager.h"

#include "hector_radiation_mapping_msgs/GPEvaluationService.h"
#include "hector_radiation_mapping_msgs/GPEvaluationServiceRequest.h"
#include "hector_radiation_mapping_msgs/GPEvaluationServiceResponse.h"
#include "hector_radiation_mapping_msgs/AddSamplesService.h"
#include "hector_radiation_mapping_msgs/AddSamplesServiceRequest.h"
#include "hector_radiation_mapping_msgs/AddSamplesServiceResponse.h"
#include "hector_radiation_mapping_msgs/Samples.h"
#include "hector_radiation_mapping_msgs/Sample.h"


GPython::GPython() : Model(ModelType::GAUSSIAN_PROCESS) {
    // Define ROS service clients
    evaluation_service_client_ = Parameters::instance().node_handle_ptr_->serviceClient<hector_radiation_mapping_msgs::GPEvaluationService>(
            "/hector_radiation_mapping/gp_evaluation");
    sample_service_client_ = Parameters::instance().node_handle_ptr_->serviceClient<hector_radiation_mapping_msgs::AddSamplesService>(
            "/hector_radiation_mapping/gp_samples");

    // Create dynamic reconfigure parameters
    param1_ptr = new double[1];
    param2_ptr = new double[1];
    param3_ptr = new double[1];
    *param1_ptr = 1.0;
    *param2_ptr = 1.0;
    *param3_ptr = 10.0;
    std::string group_name = getShortModelName();
    DDDynamicReconfigure::instance().registerVariable<double>(group_name + "_kernel_lengthscale", param1_ptr,
                                                              boost::bind(&GPython::paramCallback, this), "param1", 0.0,
                                                              4.0, group_name);
    DDDynamicReconfigure::instance().registerVariable<double>(group_name + "_outputscale", param2_ptr,
                                                              boost::bind(&GPython::paramCallback, this), "param2", 0.0,
                                                              20.0, group_name);
    DDDynamicReconfigure::instance().registerVariable<double>(group_name + "_likelihood_noise", param3_ptr,
                                                              boost::bind(&GPython::paramCallback, this), "param3", 0.0,
                                                              20.0, group_name);
    STREAM_DEBUG("GPython params: " << *param1_ptr << " " << *param2_ptr << " " << *param3_ptr);

    // Start the GPython node
    const char *command = "rosrun hector_gaussian_process gp_node &";
    int result = system(command);
    if (result != 0) {
        // Handle error
    }

    // Initialize variables
    param_update_ = false;
    model_size_ = 0;
}

void GPython::shutDown() {
    deactivate();
    GPython2D::instance().shutDown();
    GPython3D::instance().shutDown();
    ROS_INFO_STREAM(modelTypeToName(model_type_) << " shut down");
}

void GPython::activate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    GPython2D::instance().activate();
    GPython3D::instance().activate();

    if (active_) return;
    this->active_ = true;
    update_thread_ = std::thread(&GPython::updateLoop, this);
    ROS_INFO_STREAM(modelTypeToName(model_type_) << " activated");
}

void GPython::deactivate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    GPython2D::instance().deactivate();
    GPython3D::instance().deactivate();

    if (!active_) return;
    this->active_ = false;
    update_condition_.notify_one();
    update_thread_.join();
    ROS_INFO_STREAM(modelTypeToName(model_type_) << " deactivated");
}

void GPython::reset() {
    // TODO: implement
    ROS_INFO_STREAM(modelTypeToName(model_type_) << " reset");
}

std::vector<GPython::SampleGP> GPython::samplesToSamplesGP(std::vector<Sample> &samples) {
    std::vector<GPython::SampleGP> samples_gp;
    samples_gp.reserve(samples.size());
    for (const Sample &sample: samples) {
        samples_gp.emplace_back(sample, true, true);
    }
    return samples_gp;
}

void GPython::update() {
    static Clock clock;
    static std::vector<SampleGP> samples_gp_queue;
    {
        std::unique_lock<std::mutex> lock{sample_queue_mtx_};
        update_condition_.wait(lock,
                               [this]() { return (!samples_add_queue_.empty()) || !active_ || param_update_; });
        if (!active_) return;
        param_update_ = false;
        GPython2D::instance().triggerEvaluation();
        GPython3D::instance().triggerEvaluation();

        // create copy of samples
        samples_gp_queue = samplesToSamplesGP(samples_add_queue_);
        samples_add_queue_.clear();
    }
    {
        std::lock_guard<std::recursive_mutex> lock{model_mtx_};
        clock.tick();
        addSamplesToModel(samples_gp_queue);
        update_times_.push_back((double) clock.tock());
    }
    update_sizes_.push_back((double) sample_ids_2d_.size());
}

GPython::GPResult GPython::evaluate(Matrix &positions) {
    // check if positions are 2D or 3D
    if (positions.cols() != 2 && positions.cols() != 3) {
        STREAM_ERROR("GPython: evaluate() positions must be 2D or 3D");
        return {{},
                {}};
    }

    hector_radiation_mapping_msgs::GPEvaluationServiceRequest req;
    hector_radiation_mapping_msgs::GPEvaluationServiceResponse res;
    std_msgs::Float64MultiArray positions_msg;
    tf::matrixEigenToMsg(positions, positions_msg);
    req.positions = positions_msg;

    //{
    //    std::lock_guard<std::mutex> lock{model_mtx_};
    evaluation_service_client_.call(req, res);
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

    Vector std_dev_vec = Util::varianceToStdDeviation(variance_vec);
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
        if (add2d) sample_ids_2d_.push_back(s.id_);
        if (add3d) sample_ids_3d_.push_back(s.id_);

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
    sample_service_client_.call(req, res);
    //}
}

bool GPython::isSampleIn2DModel(const Sample &sample) {
    return std::find(sample_ids_2d_.begin(), sample_ids_2d_.end(), sample.id_) != sample_ids_2d_.end();
}

bool GPython::isSampleIn3DModel(const Sample &sample) {
    return std::find(sample_ids_3d_.begin(), sample_ids_3d_.end(), sample.id_) != sample_ids_3d_.end();
}

void GPython::paramCallback() {
    STREAM_DEBUG("GPython params changed " << *param1_ptr << " " << *param2_ptr << " " << *param3_ptr);
    param_update_ = true;
    update_condition_.notify_one();
}
