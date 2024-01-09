#ifndef RADIATION_MAPPING_GP_PY_MANAGER_H
#define RADIATION_MAPPING_GP_PY_MANAGER_H

#include "pch.h"
#include "hector_radiation_mapping/sample.h"
#include "models/model.h"

/**
 * @brief The GPython class
 * This class is a singleton and manages the connection to the GPython model over in Python. It provides methods to add
 * samples to the model and to evaluate the model at given positions and check if a sample is already in the model.
 * Runs in a separate thread.
 */
class GPython : public Model {
public:
    /**
     * Struct for storing the result of a GP evaluation.
     */
    struct GPResult {
        Vector mean;
        Vector std_dev;
    };

    /**
     * Struct for locally storing a sample and the information if it should be added to the 2D or 3D model.
     */
    struct SampleGP {
        SampleGP(Sample sample, bool for2d, bool for3d) : sample(std::move(sample)), for2d(for2d), for3d(for3d) {}

        Sample sample;
        bool for2d;
        bool for3d;
    };

    /**
     * Returns the instance of the GPython class.
     * @return The instance of the GPython class.
     */
    static GPython &instance(){
        static GPython instance;
        return instance;
    }

    /**
     * Resets the model.
     */
    void reset() override;

    /**
     * Deactivates the GPython class.
     */
    void deactivate() override;

    /**
     * Activates the GPython class.
     */
    void activate() override;

    /**
     * Shuts down the GPython class.
     */
    void shutDown() override;

    static std::vector<SampleGP> samplesToSamplesGP(std::vector<Sample> &samples);

    /**
     * Adds all samples within a radius to the model.
     * @param position The position.
     * @param radius The radius.
     * @param for2d If the samples should be added to the 2D model.
     * @param for3d If the samples should be added to the 3D model.
     */
    void addSamplesWithinRadius(const Vector3d &position, double radius, bool for2d, bool for3d);

    /**
     * Evaluates the model at the given positions.
     * @param positions The positions. The columns contain x, y and z coordinates. Only 2D and 3D is allowed.
     * @return The result of the evaluation. Empty if the evaluation failed or the dimension of the positions is not 2 or 3.
     */
    GPResult evaluate(Matrix &positions);

    /**
     * Get all sample ids of the 2D model
     * @return The sample ids of the 2D model
     */
    std::vector<int> getSampleIds2d() { return sample_ids_2d_; };

    /**
     * Get all sample ids of the 3D model
     * @return The sample ids of the 3D model
     */
    std::vector<int> getSampleIds3d() { return sample_ids_3d_; };

private:
    GPython();
    GPython(const GPython &) = delete;
    GPython &operator=(const GPython &) = delete;

    /**
     * Updates the model in a loop. It is called in a separate thread.
     * It sends all samples in the samples_add_queue_ to the GPython model.
     * It also updates the parameters of the GPython model.
     */
    void updateLoop() override;

    /**
     * Sends a vector of samples to the GPython model via the sample_service_client_.
     * @param samples The vector of samples to send.
     */
    void addSamplesToModel(const std::vector<SampleGP> &samples);

    /**
     * Checks if a sample is already in the 2D model.
     * @param sample The sample to check.
     * @return True if the sample is already in the 2D model.
     */
    bool isSampleIn2DModel(const Sample &sample);

    /**
     * Checks if a sample is already in the 3D model.
     * @param sample The sample to check.
     * @return True if the sample is already in the 3D model.
     */
    bool isSampleIn3DModel(const Sample &sample);

    /**
     * Callback for the dynamic reconfigure parameters. Sets the param_update_ flag that signals that the parameters
     * have changed. The parameters are updated in the updateLoop().
     */
    void paramCallback();

    // ROS
    ros::ServiceClient evaluation_service_client_;
    ros::ServiceClient sample_service_client_;

    // Model
    int model_size_;
    std::vector<int> sample_ids_2d_;
    std::vector<int> sample_ids_3d_;

    // Dynamic reconfigure parameters
    double *param1_ptr;
    double *param2_ptr;
    double *param3_ptr;
    volatile bool param_update_;

    // For runtime evaluation
    std::vector<double> update_times_;
    std::vector<double> update_sizes_;
};

#endif //RADIATION_MAPPING_GP_PY_MANAGER_H
