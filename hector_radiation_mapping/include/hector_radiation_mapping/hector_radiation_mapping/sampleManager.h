#ifndef RADIATION_MAPPING_SAMPLE_MANAGER_H
#define RADIATION_MAPPING_SAMPLE_MANAGER_H

#include <Eigen/Core>
#include <tf2_ros/transform_listener.h>
#include <ros_babel_fish/babel_fish_message.h>
#include <ros_babel_fish/babel_fish.h>

#include "sample.h"
#include "marker/marker.h"


class SampleManager {
public:
    /**
     * Returns the instance of the SampleManager class.
     * @return The instance of the SampleManager class.
     */
    static SampleManager& instance();

    /**
     * Callback for the babel fish topic to receive samples.
     * @param msg
     */
    void doseCallbackFish(const ros_babel_fish::BabelFishMessage::ConstPtr &msg);

    /**
     * Processes the given sample. It adds the sample to the sample queue and updates the background radiation.
     * @param newSample The sample to process.
     */
    void processSampleData(Vector3d pos, double cps, double dose_rate, ros::Time time);

    /**
     * Adds the given sample to the radiation models.
     * @param sample The sample to add.
     */
    void addSample(Sample& sample);

    /**
     * Updates the trajectory marker with the given value and position of a new sample.
     * @param value radiation value of sample
     * @param position position of sample
     */
    void updateTrajectory(const double &value, const Eigen::Vector3d &position);

    /**
     * Returns the samples that are within the given radius of the given position.
     * @param position position to search from
     * @param radius radius to search in
     * @return vector of samples that are within the given radius of the given position
     */
    std::vector<Sample> getSamplesWithinRadius(const Eigen::Vector3d& position, double radius);

    /**
     * Get all samples.
     * @return all samples
     */
    std::vector<Sample> getSamples();

    /**
     * Get the TF buffer.
     * @return The TF buffer.
     */
    tf2_ros::Buffer& getTfBuffer();

    /**
     * Get the position of the latest sample.
     * @return The position of the latest sample.
     */
    Eigen::Vector3d getLastSamplePos();

    /**
     * Get the mean sample of the given samples.
     * @param samples
     * @param weighted
     * @return The mean sample of the given samples.
     */
    Sample getMeanSample(const std::vector<Sample>& samples, bool weighted = false);

    /**
     * Get the Background Radiation in Cps
     * @return The Background Radiation in Cps
     */
    double getBackgroundRadiationCps(){return background_radiation_cps_;};

    /**
     * Get the Background Radiation as Dose Rate
     * @return The Background Radiation as Dose Rate
     */
    double getBackgroundRadiationDoseRate(){return background_radiation_dose_rate_;};

    /**
     * Returns the nearest sample to the given position
     * @param position position to search from
     * @return nearest sample to the given position
     */
    Sample getNearestSample(const Eigen::Vector3d& position);

    /**
     * Returns the samples that are not in the given vector of ids
     * @param samples vector of samples to search from
     * @param ids vector of ids to exclude
     * @return vector of samples that are not in the given vector of ids
     */
    std::vector<Sample> getSamplesNotInIdVector(const std::vector<Sample> &samples, const std::vector<int> &ids);

    Sample getSampleById(int id);

private:
    SampleManager();
    SampleManager(const SampleManager&) = delete;
    SampleManager& operator=(const SampleManager&) = delete;

    double background_radiation_cps_;
    double background_radiation_dose_rate_;

    std::vector<Sample> samples_;
    std::vector<Sample> sample_queue_;
    std::shared_ptr<TrajectoryMarker> trajectory_marker_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<ros::Subscriber> universal_sub_;
    std::shared_ptr<ros_babel_fish::BabelFish> babel_fish_;
};

#endif //RADIATION_MAPPING_SAMPLE_MANAGER_H
