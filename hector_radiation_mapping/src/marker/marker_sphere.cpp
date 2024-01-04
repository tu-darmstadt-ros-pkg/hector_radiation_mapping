#include "pch.h"
#include "marker/marker.h"
#include "marker/marker_manager.h"

SphereMarker::SphereMarker(const Eigen::Vector3d &origin, const Eigen::Vector4d &color) : Marker() {
    marker_.type = visualization_msgs::Marker::SPHERE;
    marker_.action = visualization_msgs::Marker::MODIFY;

    setPos(origin.x(), origin.y(), origin.z(), false);
    setColor(color.x(), color.y(), color.z(), color.w(), false);
    setScale(0.1, 0.1, 0.1, false);
    setOrientation(0.0, 0.0, 0.0, 1.0, false);
    MarkerManager::instance().publishMarker(marker_);
}