#include "marker/marker.h"
#include "marker/marker_manager.h"
#include "util/color.h"
#include "util/util.h"
#include "pch.h"

TrajectoryMarker::TrajectoryMarker(const Eigen::Vector3d &origin, const Eigen::Vector4d &color) : Marker() {
    marker_.header.frame_id = "world";
    marker_.header.stamp = ros::Time();
    marker_.type = visualization_msgs::Marker::LINE_STRIP;
    marker_.action = visualization_msgs::Marker::MODIFY;
    marker_.id = id_;

    setPos(origin.x(), origin.y(), origin.z(), false);
    setColor(color.x(), color.y(), color.z(), color.w(), false);
    setScale(0.025, 0.025, 0.025, false);
    setOrientation(0.0, 0.0, 0.0, 1.0, false);
}

void TrajectoryMarker::addPoints(const Vector &newValues, const Eigen::Matrix3Xd &newPositions) {
    Util::appendToVector(values_, newValues);
    Eigen::MatrixX4d colors = Color::applyColorMap(turbo_colormap, values_);
    marker_.colors.clear();
    for (int i = 0; i < colors.rows(); i++) {
        std_msgs::ColorRGBA color;
        color.r = colors.row(i).x();
        color.g = colors.row(i).y();
        color.b = colors.row(i).z();
        color.a = colors.row(i).w();
        marker_.colors.push_back(color);
    }
    for (int i = 0; i < newPositions.cols(); i++) {
        geometry_msgs::Point point;
        point.x = newPositions.col(i).x();
        point.y = newPositions.col(i).y();
        point.z = newPositions.col(i).z();
        marker_.points.push_back(point);
    }
    if (marker_.points.size() > 1) {
        MarkerManager::instance().publishMarker(marker_);
    }
}

void TrajectoryMarker::addPoint(const double &value, const Eigen::Vector3d &position) {
    Vector newValue(1);
    newValue << value;
    Util::appendToVector(values_, newValue);

    Eigen::MatrixX4d colors = Color::applyColorMap(turbo_colormap, values_);
    marker_.colors.clear();
    for (int i = 0; i < colors.rows(); i++) {
        std_msgs::ColorRGBA color;
        color.r = colors.row(i).x();
        color.g = colors.row(i).y();
        color.b = colors.row(i).z();
        color.a = colors.row(i).w();
        marker_.colors.push_back(color);
    }
    geometry_msgs::Point point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
    marker_.points.push_back(point);
    if (marker_.points.size() > 1) {
        MarkerManager::instance().publishMarker(marker_);
    }
}
