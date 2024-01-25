#include <hector_radiation_mapping/sampleManager.h>
#include "util/dddynamic_reconfigure.h"
#include "models/model.h"

Model::Model(Model::ModelType model_type, bool on_start_up, int min_update_time) :
        model_type_(model_type),
        min_update_time_(min_update_time),
        on_start_up_(on_start_up),
        active_(false) {
    ROS_INFO_STREAM(modelTypeToName(model_type_) << " created");
    DDDynamicReconfigure::instance().registerVariable<int>(getShortModelName() + "_minUpdateTime",
                                                           min_update_time_,
                                                           boost::bind(&Model::setMinUpdateTime, this, _1),
                                                           "min/max",
                                                           0, 5000, getShortModelName());
}

bool Model::isActive() const {
    return active_;
}

void Model::activate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (active_) return;
    this->active_ = true;
    update_thread_ = std::thread(&Model::updateLoop, this);
    ROS_INFO_STREAM(modelTypeToName(model_type_) << " activated");
}

void Model::deactivate() {
    std::lock_guard<std::mutex> lock{activation_mtx_};
    if (!active_) return;
    this->active_ = false;
    update_condition_.notify_one();
    update_thread_.join();
    ROS_INFO_STREAM(modelTypeToName(model_type_) << " deactivated");
}

void Model::shutDown() {
    deactivate();
    ROS_INFO_STREAM(modelTypeToName(model_type_) << " shut down");
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
        samples_delete_queue_.push_back(sample);
    }
    update_condition_.notify_one();
}

void Model::deleteSamples(std::vector<Sample> &samples) {
    {
        std::lock_guard<std::mutex> lock{sample_queue_mtx_};
        samples_delete_queue_.insert(samples_delete_queue_.end(), samples.begin(), samples.end());
    }
    update_condition_.notify_one();
}

void Model::updateSamples() {
    std::lock_guard<std::mutex> lock{sample_queue_mtx_};
    samples_new_.clear();
    // ersase all samples from the add queue and samples_ if they are in the delete queue
    samples_add_queue_.erase(std::remove_if(samples_add_queue_.begin(), samples_add_queue_.end(),
                                            [this](const Sample &sample_add) {
                                                return std::any_of(samples_delete_queue_.begin(),
                                                                   samples_delete_queue_.end(),
                                                                   [&sample_add](const Sample &sample_delete) {
                                                                       return sample_delete.id_ == sample_add.id_;
                                                                   });
                                            }),
                             samples_add_queue_.end());
    samples_.erase(std::remove_if(samples_.begin(), samples_.end(), [this](const Sample &sample) {
                       return std::any_of(samples_delete_queue_.begin(), samples_delete_queue_.end(),
                                          [&sample](const Sample &sample_delete) { return sample_delete.id_ == sample.id_; });
                   }),
                   samples_.end());
    samples_delete_queue_.clear();

    // insert all samples from the add queue if not already present
    for (auto &sample_add: samples_add_queue_) {
        auto it = std::find_if(samples_.begin(), samples_.end(),
                               [&sample_add](const Sample &sample) {
                                   return sample_add.id_ == sample.id_;
                               });

        if (it == samples_.end()) {
            samples_.push_back(sample_add);
            samples_new_.push_back(sample_add);
        }
    }
    samples_add_queue_.clear();
}

void Model::updateLoop() {
    Clock clock;
    int sleepTime;
    while (active_ && ros::ok()) {
        clock.tick();
        update();
        sleepTime = std::max(0, min_update_time_ - (int) clock.tock());
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
    }
}

void Model::setMinUpdateTime(int time) {
    STREAM(modelTypeToName(model_type_) + " set min map update time = " << time);
    min_update_time_ = time;
}

