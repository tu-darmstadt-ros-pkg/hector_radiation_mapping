#ifndef RADIATION_MAPPING_POINTCLOUD3D_H
#define RADIATION_MAPPING_POINTCLOUD3D_H

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "pcl_ros/point_cloud.h"

/**
 * @brief The PointCloud3D class
 * Provides methods to generate the point cloud from the environment cloud and to publish the point cloud.
 */
class PointCloud3D {
public:
    struct PointXYZPC {
        PCL_ADD_POINT4D;
        float prediction;
        float std_deviation;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    /**
     * Constructor. Sets the name of the point cloud.
     * @param name
     */
    explicit PointCloud3D(const std::string &name);

    /**
     * Publishes the point cloud.
     */
    void publish();

    /**
     * Updates the values of the point cloud with the given predictions and standard deviations.
     * @param predictions
     * @param std_deviation
     */
    void updateMapValues(const Vector &predictions, const Vector &std_deviation);

    /**
     * Generates the point cloud from the given environment cloud and its transform, around the given position within
     * the given radius from the parameter server. It has three layers with different step sizes to generate a
     * point cloud with a higher density around the given position.
     * @param environmentCloud
     * @param environmentCloudTransform
     * @param pos
     */
    void generatePointCloudFromEnvironmentCloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& environmentCloud,
                                                const geometry_msgs::TransformStamped &environmentCloudTransform,
                                                const Eigen::Vector3d& pos);

    /**
     * Generates the point cloud from the given environment cloud and its transform, with the given step size, for the
     * whole environment cloud.
     * @param environmentCloud
     * @param environmentCloudTransform
     * @param stepSize step size to use to downsample the environment cloud
     */
    void generatePointCloudFromEnvironmentCloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& environmentCloud,
                                                const geometry_msgs::TransformStamped &environmentCloudTransform,
                                                int stepSize);

    /**
     * Transforms the given point with the given transform.
     * @param point
     * @param transform
     * @return The transformed point.
     */
    static PointCloud3D::PointXYZPC transformPoint(const pcl::PointXYZ& point, const geometry_msgs::TransformStamped &transform);

    /**
     * Get the sample positions of the point cloud.
     * @return The sample positions of the point cloud as a matrix. The columns contain x, y and z coordinates.
     */
    Matrix getMapSamplePositions();

    /**
     * Get the point cloud.
     * @return The point cloud.
     */
    pcl::PointCloud<PointXYZPC> &getPointCloud();

    /**
     * Get the radius of the point cloud.
     * @return The radius of the point cloud.
     */
    double getPointCloudRadius();

    /**
     * Get the name of the point cloud.
     * @return The name of the point cloud.
     */
    std::string getName() { return name_; };

    /**
     * Get the step size of the point cloud. This is the step size used to downsample the environment cloud in its
     * outer layer.
     * @return The step size of the point cloud.
     */
    double getStepSize() { return stepSize_; };

    /**
     * Get the mutex of the point cloud.
     * @return The mutex of the point cloud.
     */
    std::mutex& getPointCloudMutex() { return mapMutex_; }

private:
    /**
     * Generates the point cloud from the given environment cloud and its transform, with the given indices and the
     * given maximum number of points.
     * @param environmentCloud
     * @param environmentCloudTransform
     * @param indices indices of the environment cloud to use
     * @param maxPoints maximum number of points to use
     */
    void generatePointCloudFromEnvironmentCloud(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& environmentCloud,
                                                const geometry_msgs::TransformStamped &environmentCloudTransform,
                                                const std::vector<int>& indices, int maxPoints);

    std::string pointCloudframe_;
    pcl::PointCloud<PointXYZPC> pointCloud_;
    double stepSize_{};
    std::string name_;

    double longDist_;
    double midDist_;
    double shortDist_;
    int longDistSize_;
    int midDistSize_;
    int shortDistSize_;

    std::mutex mapMutex_;
    ros::Publisher mapPublisher_;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointCloud3D::PointXYZPC,
                                  (float, x, x)(float, y, y)(float, z, z)(float, prediction, prediction)(float,
                                                                                                         std_deviation,
                                                                                                         std_deviation))
#endif  // RADIATION_MAPPING_POINTCLOUD3D_H