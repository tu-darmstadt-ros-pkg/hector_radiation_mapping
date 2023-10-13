#ifndef RADIATION_MAPPING_GP_PY_MANAGER_H
#define RADIATION_MAPPING_GP_PY_MANAGER_H

#include <utility>
#include "hector_radiation_mapping/sample.h"
#include "pch.h"

/**
 * @brief The GPython class
 * This class is a singleton and manages the connection to the GPython model over in Python. It provides methods to add
 * samples to the model and to evaluate the model at given positions and check if a sample is already in the model.
 * Runs in a separate thread.
 */
class GPython {
public:
    /**
     * Struct for storing the result of a GP evaluation.
     */
    struct GPResult {
        Vector mean;
        Vector stdDev;
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
    static GPython &instance();

    /**
     * Returns if the GPython class is active.
     * @return True if the GPython class is active.
     */
    bool isActive() const;

    /**
     * Deactivates the GPython class.
     */
    void deactivate();

    /**
     * Activates the GPython class.
     */
    void activate();

    /**
     * Shuts down the GPython class.
     */
    void shutDown();

    /**
     * Adds a sample to the model.
     * @param sample The sample to add.
     */
    void addSample(SampleGP &sample);

    /**
     * Adds a vector of samples to the model.
     * @param samples The vector of samples to add.
     */
    void addSamples(std::vector<SampleGP> &samples);

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
    std::vector<int> getSampleIds2d() { return sampleIds2d_; };

    /**
     * Get all sample ids of the 3D model
     * @return The sample ids of the 3D model
     */
    std::vector<int> getSampleIds3d() { return sampleIds3d_; };

    /**
     * Get the mutex for the model
     * @return The mutex for the model
     */
    std::mutex &getModelMutex() { return model_mtx_; };

private:
    GPython();
    GPython(const GPython &) = delete;
    GPython &operator=(const GPython &) = delete;

    /**
     * Converts a vector of variances to a vector of standard deviations.
     * @param variance The vector of variances.
     * @return The vector of standard deviations.
     */
    static Vector varianceToStdDeviation(Vector &variance);

    /**
     * Updates the model in a loop. It is called in a separate thread.
     * It sends all samples in the samplesQueue_ to the GPython model.
     * It also updates the parameters of the GPython model.
     */
    void updateLoop();

    /**
     * Sends a vector of samples to the GPython model via the sampleServiceClient_.
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
     * Callback for the dynamic reconfigure parameters. Sets the paramUpdate_ flag that signals that the parameters
     * have changed. The parameters are updated in the updateLoop().
     */
    void paramCallback();

    // ROS
    ros::ServiceClient evaluationServiceClient_;
    ros::ServiceClient sampleServiceClient_;

    // Model
    std::vector<SampleGP> samplesQueue_;
    std::vector<int> sampleIds2d_;
    std::vector<int> sampleIds3d_;

    // Dynamic reconfigure parameters
    std::string groupName_;
    double *param1_ptr;
    double *param2_ptr;
    double *param3_ptr;
    volatile bool paramUpdate_;

    // For runtime evaluation
    std::vector<double> updateTimes_;
    std::vector<double> updateSizes_;

    // For thread management
    volatile bool active_;
    std::thread updateThread_;
    std::condition_variable waitCondition_;
    std::mutex activation_mtx_;
    std::mutex sampleQueue_mtx_;
    std::mutex model_mtx_;
};

#endif //RADIATION_MAPPING_GP_PY_MANAGER_H
