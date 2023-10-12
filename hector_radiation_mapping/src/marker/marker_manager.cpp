#include "marker/marker_manager.h"
#include "util/parameters.h"

MarkerManager::MarkerManager() {
    nodeHandle_ = Parameters::instance().nodeHandle_;
    markerPublisher_ = std::make_shared<ros::Publisher>(
            nodeHandle_->advertise<visualization_msgs::Marker>("marker", 1000));
    interactiveMarkerServer_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("source_markers");
}

MarkerManager &MarkerManager::instance() {
    static MarkerManager instance;
    return instance;
}

void MarkerManager::publishMarker(visualization_msgs::Marker &marker) {
    markerPublisher_->publish(marker);
}

void MarkerManager::deleteAllMarkers() {
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    publishMarker(marker);
}

void MarkerManager::deleteMarker(visualization_msgs::Marker &marker) {
    marker.action = visualization_msgs::Marker::DELETE;
    publishMarker(marker);
}

void MarkerManager::deleteAllInteractiveMarkers() {
    interactiveMarkerServer_.reset();
    interactiveMarkerServer_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("source_markers");
}