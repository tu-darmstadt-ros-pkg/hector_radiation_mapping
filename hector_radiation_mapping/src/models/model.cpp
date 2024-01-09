#include <hector_radiation_mapping/sampleManager.h>
#include "models/model.h"


bool Model::isActive() const {
    return active_;
}

void Model::activate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (active_) return;
    this->active_ = true;
    update_thread_ = std::thread(&Model::updateLoop, this);
    ROS_INFO_STREAM(getModelTypeName(model_type_) << " activated");
}

void Model::deactivate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (!active_) return;
    this->active_ = false;
    update_condition_.notify_one();
    update_thread_.join();
    ROS_INFO_STREAM(getModelTypeName(model_type_) << " deactivated");
}

void Model::shutDown() {
    {
        std::lock_guard<std::mutex> lock{sample_queue_mtx_};
        samples_add_queue_.clear();
    }
    deactivate();
    ROS_INFO_STREAM(getModelTypeName(model_type_) << " shut down");
}

void Model::addSample(Sample &sample) {
    {
        std::lock_guard<std::mutex> lock{sample_queue_mtx_};
        samples_add_queue_.push_back(sample);
    }
    update_condition_.notify_one();
}

void Model::addSamples(std::vector<Sample> &samples) {
    {
        std::lock_guard<std::mutex> lock{sample_queue_mtx_};
        samples_add_queue_.insert(samples_add_queue_.end(), samples.begin(), samples.end());
    }
    update_condition_.notify_one();
}

void Model::deleteSample(Sample &sample) {
    {
        std::lock_guard<std::mutex> lock{sample_queue_mtx_};
        // remove sample from samples_add_queue_ if it is there
        samples_add_queue_.erase(std::remove_if(samples_add_queue_.begin(), samples_add_queue_.end(),
                                                [&sample](const Sample &s) { return s.id_ == sample.id_; }),
                                 samples_add_queue_.end());

        samples_delete_queue_.push_back(sample);
    }
    update_condition_.notify_one();
}

void Model::deleteSamples(std::vector<Sample> &samples) {
    {
        std::lock_guard<std::mutex> lock{sample_queue_mtx_};
        // remove samples from samples_add_queue_ if they are there
        for (const Sample &sample : samples) {
            samples_add_queue_.erase(std::remove_if(samples_add_queue_.begin(), samples_add_queue_.end(),
                                                    [&sample](const Sample &s) { return s.id_ == sample.id_; }),
                                     samples_add_queue_.end());
        }
        samples_delete_queue_.insert(samples_delete_queue_.end(), samples.begin(), samples.end());
    }
    update_condition_.notify_one();
}
