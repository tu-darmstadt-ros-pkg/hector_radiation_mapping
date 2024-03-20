#ifndef RADIATION_MAPPING_MODEL_H
#define RADIATION_MAPPING_MODEL_H

#include "pch.h"

#include "hector_radiation_mapping/sample.h"

class Model {
public:

    enum ModelType {
        GAUSSIAN_PROCESS,
        LEAST_SQUARES,
        FIELD_PROPAGATION,
        BAYESIAN_INFERENCE,
        TRIANGULATION,
        TRI_RSD,
        MLE,
        SPRT,
        PF,
        SRD,
        ROSD_RSD
        // Add more model types here
    };

    static std::string modelTypeToName(ModelType type, bool short_name = false) {
        if (short_name) {
            switch (type) {
                case GAUSSIAN_PROCESS: return "GP";
                case LEAST_SQUARES: return "LS";
                case FIELD_PROPAGATION: return "FP";
                case BAYESIAN_INFERENCE: return "BI";
                case TRIANGULATION: return "TR";
                case TRI_RSD: return "TRI";
                case MLE: return "MLE";
                case SPRT: return "SPRT";
                case PF: return "PF";
                case SRD: return "SRD";
                case ROSD_RSD: return "ROS";
                // Add more model types here
                default: return "UNKNOWN";
            }
        } else {
            switch (type) {
                case GAUSSIAN_PROCESS: return "Gaussian_Process";
                case LEAST_SQUARES: return "Least_Squares";
                case FIELD_PROPAGATION: return "Field_Propagation";
                case BAYESIAN_INFERENCE: return "Bayesian_Inference";
                case TRIANGULATION: return "Triangulation";
                case TRI_RSD: return "Tri_RSD";
                case MLE: return "Maximum_Likelihood_Estimation";
                case SPRT: return "Sequential_Probability_Ratio_Test";
                case PF: return "Particle_Filter";
                case SRD: return "Source_Attractor_Radiation_Detection";
                case ROSD_RSD: return "Ratio_of_Squared_Distances_Radiation_Source_Detection";
                // Add more model types here
                default: return "UNKNOWN";
            }
        }
    }

    std::string getModelName() {
        return modelTypeToName(model_type_);
    }

    std::string getShortModelName() {
        return modelTypeToName(model_type_, true);
    }

    ModelType getModelType() {
        return model_type_;
    }

    /**
     * Get the mutex for the model
     * @return The mutex for the model
     */
    std::recursive_mutex &getModelMutex() { return model_mtx_; };

    /**
     * Returns if the GaussianProcess class is active.
     * @return True if the GaussianProcess class is active.
     */
    bool isActive() const;

    /**
     * Deactivates the GaussianProcess class.
     */
    virtual void deactivate();

    /**
     * Activates the GaussianProcess class.
     */
    virtual void activate();

    /**
     * Shuts down the GaussianProcess class.
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

    bool onStartup() {
        return on_start_up_;
    }

protected:
    explicit Model(Model::ModelType model_type, bool on_start_up, int min_update_time = 0);

    virtual void updateLoop();

    virtual void update() = 0;

    virtual void updateSamples();

    /**
     * Set the minimum update time.
     * @param time
     */
    void setMinUpdateTime(int time);

    std::vector<Sample> samples_;
    std::vector<Sample> samples_new_;
    std::vector<Sample> samples_add_queue_;
    std::vector<Sample> samples_delete_queue_;
    std::mutex sample_queue_mtx_;

    volatile std::atomic_bool active_;
    volatile std::atomic_bool on_start_up_;
    volatile std::atomic_int min_update_time_;
    std::thread update_thread_;
    std::condition_variable update_condition_;
    std::mutex activation_mtx_;

    std::recursive_mutex model_mtx_;
    ModelType model_type_;
};

#endif //RADIATION_MAPPING_MODEL_H