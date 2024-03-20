#include "marker/marker.h"
#include "marker/marker_manager.h"

ArrowMarker::ArrowMarker(const Eigen::Vector3d &origin, const Eigen::Vector3d &direction, const Eigen::Vector4d &color) : Marker() {
    marker_.type = visualization_msgs::Marker::ARROW;
    marker_.action = visualization_msgs::Marker::MODIFY;

    setPos(origin.x(), origin.y(), origin.z(), false);
    setColor(color.x(), color.y(), color.z(), color.w(), false);
    setScale(0.06, 0.1, 0.1, false);
    setOrientation(0.0, 0.0, 0.0, 1.0, false);
    setDirection(direction, false);
    MarkerManager::instance().publishMarker(marker_);
}

void ArrowMarker::setDirection(const Vector3d &direction, bool publish) {
    marker_.points.clear();
    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    p.z = 0;
    marker_.points.push_back(p);
    p.x = direction.x();
    p.y = direction.y();
    p.z = direction.z();
    marker_.points.push_back(p);
    if (publish) {
        MarkerManager::instance().publishMarker(marker_);
    }
}
