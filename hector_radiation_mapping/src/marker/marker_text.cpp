#include "marker/marker.h"
#include "marker/marker_manager.h"

TextMarker::TextMarker(const Eigen::Vector3d &origin, const std::string &text, double zOffset,
                       const Eigen::Vector4d &color) : Marker(), sphere(origin, color) {
    marker_.header.frame_id = "world";
    marker_.header.stamp = ros::Time();
    marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_.action = visualization_msgs::Marker::MODIFY;
    marker_.text = text;
    marker_.id = id_;
    zOffset_ = zOffset;

    setPos(origin.x(), origin.y(), origin.z(), false);
    setColor(color.x(), color.y(), color.z(), color.w(), false);
    setScale(0.1, 0.1, 0.1, false);
    setOrientation(0.0, 0.0, 0.0, 1.0, false);
    MarkerManager::instance().publishMarker(marker_);
}

void TextMarker::setPos(const Eigen::Vector3d &pos, bool publish) {
    marker_.pose.position.x = pos.x();
    marker_.pose.position.y = pos.y();
    marker_.pose.position.z = pos.z() + zOffset_;
    sphere.setPos(pos, publish);
    if (publish) { MarkerManager::instance().publishMarker(marker_); }
}

void TextMarker::setPos(double x, double y, double z, bool publish) {
    Marker::setPos(x, y, z + zOffset_, publish);
    if (publish) { MarkerManager::instance().publishMarker(marker_); }
}

void TextMarker::setzOffset(double zOffset) {
    marker_.pose.position.z += zOffset - zOffset_;
    MarkerManager::instance().publishMarker(marker_);
}

void TextMarker::setText(const std::string &text) {
    marker_.text = text;
    MarkerManager::instance().publishMarker(marker_);
}

void TextMarker::deleteMarker() {
    sphere.deleteMarker();
    Marker::deleteMarker();
}

std::string TextMarker::getText() {
    return marker_.text;
}