#include "marker/marker.h"
#include "marker/marker_manager.h"
#include "pch.h"

Marker::Marker() {
    id_ = idCounter_;
    idCounter_++;
}

void Marker::deleteMarker() {
    MarkerManager::instance().deleteMarker(marker_);
}

void Marker::setColor(double r, double g, double b, double a, bool publish) {
    marker_.color.r = r;
    marker_.color.g = g;
    marker_.color.b = b;
    marker_.color.a = a;
    if (publish) { MarkerManager::instance().publishMarker(marker_); }
}

void Marker::setColor(const Eigen::Vector4d &color, bool publish) {
    marker_.color.r = color.x();
    marker_.color.g = color.y();
    marker_.color.b = color.z();
    marker_.color.a = color.w();
    if (publish) { MarkerManager::instance().publishMarker(marker_); }
}

void Marker::setPos(double x, double y, double z, bool publish) {
    marker_.pose.position.x = x;
    marker_.pose.position.y = y;
    marker_.pose.position.z = z;
    if (publish) { MarkerManager::instance().publishMarker(marker_); }
}

void Marker::setPos(const Eigen::Vector3d &pos, bool publish) {
    marker_.pose.position.x = pos.x();
    marker_.pose.position.y = pos.y();
    marker_.pose.position.z = pos.z();
    if (publish) { MarkerManager::instance().publishMarker(marker_); }
}

void Marker::setOrientation(double x, double y, double z, double w, bool publish) {
    marker_.pose.orientation.x = x;
    marker_.pose.orientation.y = y;
    marker_.pose.orientation.z = z;
    marker_.pose.orientation.w = w;
    if (publish) { MarkerManager::instance().publishMarker(marker_); }
}

void Marker::setOrientation(Eigen::Vector4d orientation, bool publish) {
    marker_.pose.orientation.x = orientation.x();
    marker_.pose.orientation.y = orientation.y();
    marker_.pose.orientation.z = orientation.z();
    marker_.pose.orientation.w = orientation.w();
    if (publish) { MarkerManager::instance().publishMarker(marker_); }
}

void Marker::setScale(double x, double y, double z, bool publish) {
    marker_.scale.x = x;
    marker_.scale.y = y;
    marker_.scale.z = z;
    if (publish) { MarkerManager::instance().publishMarker(marker_); }
}

void Marker::setScale(const Eigen::Vector3d &scale, bool publish) {
    marker_.scale.x = scale.x();
    marker_.scale.y = scale.y();
    marker_.scale.z = scale.z();
    if (publish) { MarkerManager::instance().publishMarker(marker_); }
}
