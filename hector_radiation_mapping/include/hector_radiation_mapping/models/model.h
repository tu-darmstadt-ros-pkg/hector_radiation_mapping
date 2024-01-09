#ifndef RADIATION_MAPPING_MODEL_H
#define RADIATION_MAPPING_MODEL_H

#include "hector_radiation_mapping/sample.h"

class Model {
public:

    enum ModelType {
        GAUSSIAN_PROCESS,
        LEAST_SQUARES,
        FIELD_PROPAGATION,
        BAYESIAN_INFERENCE
        // Add more model types here
    };

    static std::string getModelTypeName(ModelType type) {
        switch (type) {
            case GAUSSIAN_PROCESS: return "GAUSSIAN_PROCESS";
            case LEAST_SQUARES: return "LEAST_SQUARES";
            case FIELD_PROPAGATION: return "FIELD_PROPAGATION";
            case BAYESIAN_INFERENCE: return "BAYESIAN_INFERENCE";
            // Add more model types here
            default: return "UNKNOWN";
        }
    }

    /**
     * Get the mutex for the model
     * @return The mutex for the model
     */
    std::recursive_mutex &getModelMutex() { return model_mtx_; };

    /**
     * Returns if the GPython class is active.
     * @return True if the GPython class is active.
     */
    bool isActive() const;

    /**
     * Deactivates the GPython class.
     */
    virtual void deactivate();

    /**
     * Activates the GPython class.
     */
    virtual void activate();

    /**
     * Shuts down the GPython class.
     */
    virtual void shutDown();

    /**
    * Adds a sample to the model.
    * @param sample The sample to add.
    */
    virtual void addSample(Sample &sample);

    /**
     * Adds a vector of samples to the model.
     * @param samples The vector of samples to add.
     */
    virtual void addSamples(std::vector<Sample> &samples);

    /**
     * Deletes a sample from the model.
     * @param sample The sample to delete.
     */
    virtual void deleteSample(Sample &sample);

    /**
     * Deletes a vector of samples from the model.
     * @param samples The vector of samples to delete.
     */
    virtual void deleteSamples(std::vector<Sample> &samples);

    virtual void reset() = 0;

protected:
    Model(ModelType model_type){
        model_type_ = model_type;
        ROS_INFO_STREAM(getModelTypeName(model_type_) << " created");
    }

    virtual void updateLoop() = 0;

    std::vector<Sample> samples_add_queue_;
    std::vector<Sample> samples_delete_queue_;
    std::mutex sample_queue_mtx_;

    volatile bool active_ = false;
    std::thread update_thread_;
    std::condition_variable update_condition_;
    std::mutex activation_mtx_;

    std::recursive_mutex model_mtx_;
    ModelType model_type_;
};

#endif //RADIATION_MAPPING_MODEL_H