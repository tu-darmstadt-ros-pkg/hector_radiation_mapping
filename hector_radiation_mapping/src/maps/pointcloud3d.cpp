#include <pcl/io/ply_io.h>

#include "maps/pointcloud3d.h"
#include "util/print.h"
#include "util/parameters.h"

PointCloud3D::PointCloud3D(const std::string &name) {
    name_ = name;
    stepSize_ = 1;
    mapPublisher_ = Parameters::instance().nodeHandle_->advertise<sensor_msgs::PointCloud2>(name_, 1, true);

    longDist_ = Parameters::instance().distanceCutoffLevels[2];
    midDist_ = Parameters::instance().distanceCutoffLevels[1];
    shortDist_ = Parameters::instance().distanceCutoffLevels[0];
    longDistSize_ = Parameters::instance().pointCloud3DSizeLevels[2];
    midDistSize_ = Parameters::instance().pointCloud3DSizeLevels[1];
    shortDistSize_ = Parameters::instance().pointCloud3DSizeLevels[0];
}

void PointCloud3D::generatePointCloudFromEnvironmentCloud(
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& environmentCloud,
        const geometry_msgs::TransformStamped &environmentCloudTransform,
        const Eigen::Vector3d &center) {
    // get indices of all points that are within maxDistance of origin
    int i = 0;
    std::vector<int> longDistIndices;
    std::vector<int> midDistIndices;
    std::vector<int> shortDistIndices;
    double shortDistSquared = shortDist_ * shortDist_;
    double midDistSquared = midDist_ * midDist_;
    double longDistSquared = longDist_ * longDist_;

    for (const pcl::PointXYZ &point: *environmentCloud) {
        Eigen::Vector3d p(point.x, point.y, point.z);
        double dist = (p - center).squaredNorm();
        if (dist < shortDistSquared) {
            shortDistIndices.push_back(i);
        } else if (dist < midDistSquared) {
            midDistIndices.push_back(i);
        } else if (dist < longDistSquared) {
            longDistIndices.push_back(i);
        }
        i++;
    }

    pointCloud_.clear();
    generatePointCloudFromEnvironmentCloud(environmentCloud, environmentCloudTransform, shortDistIndices, shortDistSize_);
    generatePointCloudFromEnvironmentCloud(environmentCloud, environmentCloudTransform, midDistIndices, midDistSize_);
    generatePointCloudFromEnvironmentCloud(environmentCloud, environmentCloudTransform, longDistIndices, longDistSize_);
    pointCloudframe_ = "world";
}

void PointCloud3D::generatePointCloudFromEnvironmentCloud(
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& environmentCloud,
        const geometry_msgs::TransformStamped &environmentCloudTransform,
        const std::vector<int> &indices, int maxPoints) {
    // downsample all Points within to a manageable max number of points
    int numPoints = indices.size();
    stepSize_ = (double) numPoints / (double) maxPoints;
    int stepSize = ceil(stepSize_);
    int index = 0;

    while (index < numPoints - stepSize) {
        pointCloud_.push_back(transformPoint(environmentCloud->at(indices[index]), environmentCloudTransform));
        index = index + stepSize;
    }
}

void PointCloud3D::generatePointCloudFromEnvironmentCloud(
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& environmentCloud,
        const geometry_msgs::TransformStamped &environmentCloudTransform,
        int stepSize) {
    pointCloud_.clear();
    int index = 0;
    int numPoints = environmentCloud->size();
    STREAM_DEBUG("Generating point cloud from environment cloud with step size " << stepSize << " and " << numPoints << " points.");
    while (index < numPoints - stepSize) {
        pointCloud_.push_back(transformPoint(environmentCloud->at(index), environmentCloudTransform));
        index = index + stepSize;
    }
    pointCloudframe_ = "world";
}

PointCloud3D::PointXYZPC PointCloud3D::transformPoint(const pcl::PointXYZ &point, const geometry_msgs::TransformStamped &transform) {
    geometry_msgs::PointStamped pointInOldFrame;
    pointInOldFrame.header.frame_id = "world";
    pointInOldFrame.header.stamp = ros::Time::now();
    pointInOldFrame.point.x = point.x;
    pointInOldFrame.point.y = point.y;
    pointInOldFrame.point.z = point.z;

    geometry_msgs::PointStamped pointInNewFrame;
    tf2::doTransform(pointInOldFrame, pointInNewFrame, transform);

    PointXYZPC newPoint{};
    newPoint.x = (float)pointInNewFrame.point.x;
    newPoint.y = (float)pointInNewFrame.point.y;
    newPoint.z = (float)pointInNewFrame.point.z;
    newPoint.prediction = 0;
    newPoint.std_deviation = 0;
    return newPoint;
}

void PointCloud3D::publish() {
    sensor_msgs::PointCloud2 pointCloudMsg;
    pointCloud_.header.frame_id = pointCloudframe_;
    pointCloudMsg.header.frame_id = pointCloudframe_;
    pointCloudMsg.header.stamp = ros::Time::now();
    pcl::toROSMsg(pointCloud_, pointCloudMsg);
    mapPublisher_.publish(pointCloudMsg);
}

Matrix PointCloud3D::getMapSamplePositions() {
    Matrix positions(pointCloud_.size(), 3);
    int index = 0;
    for (PointXYZPC &point: pointCloud_) {
        positions.row(index) << point.x, point.y, point.z;
        index++;
    }
    return positions;
}

void PointCloud3D::updateMapValues(const Vector &predictions, const Vector &std_deviation) {
    for (int i = 0; i < predictions.size(); i++) {
        pointCloud_[i].prediction = (float)predictions(i);
        pointCloud_[i].std_deviation = (float)std_deviation(i);
    }
}

pcl::PointCloud<PointCloud3D::PointXYZPC> &PointCloud3D::getPointCloud() {
    return pointCloud_;
}

double PointCloud3D::getPointCloudRadius() {
    return longDist_;
}