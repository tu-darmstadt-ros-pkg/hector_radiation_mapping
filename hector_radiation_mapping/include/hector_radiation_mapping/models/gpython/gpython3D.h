#ifndef RADIATION_MAPPING_GPYTHON3D_H
#define RADIATION_MAPPING_GPYTHON3D_H

#include "gpython.h"
#include "marker/marker.h"
#include "maps/pointcloud3d.h"
#include "hector_radiation_mapping/source.h"
#include "hector_radiation_mapping/source_interactive.h"

/**
 * @brief The GPython3D class is a singleton and manages the 3D PointCloud, the evaluation process of its positions in the
 * GPython model and the localisation of the sources. Runs in a separate thread.
 */
class GPython3D {
public:
    /**
     * Returns the instance of the GPython3D class.
     * @return The instance of the GPython3D class.
     */
    static GPython3D &instance();

    /**
     * Returns if the GPython3D class is active.
     * @return True if the GPython3D class is active.
     */
    bool isActive() { return active_; };

    /**
     * Deactivates the GPython3D class.
     */
    void deactivate();

    /**
     * Activates the GPython3D class.
     */
    void activate();

    /**
     * Shuts down the GPython3D class.
     */
    void shutDown();

    /**
     * Returns the point cloud.
     * @return The point cloud.
     */
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getEnvironmentCloud() { return environmentCloud_; };

    /**
     * Returns the transform of the point cloud.
     * @return The transform of the point cloud.
     */
    geometry_msgs::TransformStamped getEnvironmentCloudTransform() { return environmentCloudTransform_; };

    /**
     * Check if the GPython3D class has a point cloud.
     * @return True if the GPython3D class has a point cloud.
     */
    bool hasEnvironmentCloud() { return environmentCloud_ != nullptr; };

    /**
     * Get the sources.
     * @return The sources.
     */
    std::vector<std::shared_ptr<Source>> getSources() { return sources_; };

    /**
     * Adds a source Candidate to the model. These Candidates are used to find the positions to evaluate the model at
     * and to find the source in 3D, as the interactive SourceCandidates come the 2D model.
     * @param sourceCandidate The source Candidates to add.
     */
    void addSourceCandidate(std::shared_ptr<SourceInteractive> &sourceCandidate);

    /**
     * Get the PointCloud of the Model used for the export.
     * @return The PointCloud of the Model used for the export.
     */
    pcl::PointCloud<PointCloud3D::PointXYZPC> getExportPointCloud();

    /**
     * Forcefully trigger the evaluation of the grid map.
     */
    void triggerEvaluation();

private:
    GPython3D();
    GPython3D(const GPython3D &) = delete;
    GPython3D &operator=(const GPython3D &) = delete;

    /**
     * The update loop of the GPython3D class. It waits for the activation signal and then evaluates the model and
     * updates the grid map.
     */
    void updateLoop();

    /**
     * Updates the PointCloud with the given mean and standard deviation.
     * @param mean The mean.
     * @param std_dev The standard deviation.
     */
    void updatePointCloud(const Vector &mean, const Vector &std_dev);

    /**
     * Get the positions to evaluate the model at.
     * @param center
     * @return The positions to evaluate the model at as a matrix. The columns contain x, y and z coordinates.
     */
    Matrix getSamplePositions(const Vector3d &center);

    /**
     * Get a source prediction for the given positions and mean values at those positions.
     * @param positions
     * @param mean
     * @return The source prediction.
     */
    std::shared_ptr<Source> getSourcePrediction(const Matrix &positions, const Vector &mean);

    /**
     * Set the minimum update time.
     * @param time
     */
    void setMinUpdateTime(int time);

    /**
     * Set if the GPython3D class should update.
     * @param update
     */
    void setUpdate(bool update);

    /**
     * Callback for the environment cloud topic. It updates the environment cloud used as PointCloud for the model.
     * @param pointCloudMsgPtr The message.
     */
    void environmentCloudCallback(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr &pointCloudMsgPtr);

    std::shared_ptr<Source> currentSource_;
    std::vector<std::shared_ptr<Source>> sources_;
    std::vector<std::shared_ptr<SourceInteractive>> sourceCandidates_;

    std::string groupName_;
    volatile int minUpdateTime_;

    std::shared_ptr<PointCloud3D> pointCloud_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> environmentCloud_;
    geometry_msgs::TransformStamped environmentCloudTransform_;
    std::shared_ptr<ros::Subscriber> environmentCloudSub_;
    volatile bool update_;

    // Threading
    volatile bool active_;
    volatile bool doEvaluation_;
    std::thread updateThread_;
    std::condition_variable waitCondition_;
    std::mutex activation_mtx_;
    std::mutex addSource_mtx_;
    std::mutex environmentCloud_mtx_;
};

#endif //RADIATION_MAPPING_GPYTHON3D_H
