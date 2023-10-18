#include <tf/tf.h>

#include "pch.h"
#include "hector_radiation_mapping/source_interactive.h"
#include "marker/marker_manager.h"

SourceInteractive::SourceInteractive(const Eigen::Vector3d &position, double cps, double doseRate,
                                     interactive_markers::InteractiveMarkerServer::FeedbackCallback callback)
        : Source(position, cps, doseRate, false), callback_(callback) {
    wasConfirmed_ = false;

    intMarker_.header.frame_id = "world";
    intMarker_.header.stamp = ros::Time::now();
    tf::Vector3 posTf(position.x(), position.y(), position.z());
    tf::pointTFToMsg(posTf, intMarker_.pose.position);

    STREAM_DEBUG("Source 0 " << name_ << "|" << description_);
    intMarker_.name = name_;
    intMarker_.description = description_;
    visualMarker_.type = visualization_msgs::Marker::SPHERE;
    visualMarker_.scale.x = 0.2;
    visualMarker_.scale.y = 0.2;
    visualMarker_.scale.z = 0.2;
    visualMarker_.color.r = 1.0;
    visualMarker_.color.g = 1.0;
    visualMarker_.color.b = 1.0;
    visualMarker_.color.a = 1.0;

    control_.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control_.name = "button";
    control_.always_visible = true;
    control_.markers.push_back(visualMarker_);
    intMarker_.controls.push_back(control_);

    MarkerManager::instance().getInteractiveMarkerServer()->insert(intMarker_, callback_);
    MarkerManager::instance().getInteractiveMarkerServer()->applyChanges();
}

SourceInteractive::~SourceInteractive() {
    MarkerManager::instance().getInteractiveMarkerServer()->erase(intMarker_.name);
    MarkerManager::instance().getInteractiveMarkerServer()->applyChanges();
}

void SourceInteractive::setConfirmed(bool confirmed) {
    if (confirmed == confirmed_) {
        return;
    }
    if (confirmed) {
        confirmed_ = true;
        wasConfirmed_ = true;
        visualMarker_.color.r = 0.0;
        visualMarker_.color.g = 1.0;
        visualMarker_.color.b = 0.0;
        visualMarker_.color.a = 1.0;
    } else {
        confirmed_ = false;
        visualMarker_.color.r = 1.0;
        visualMarker_.color.g = 1.0;
        visualMarker_.color.b = 1.0;
        visualMarker_.color.a = 1.0;
    }
    control_.markers.clear();
    control_.markers.push_back(visualMarker_);
    intMarker_.controls.clear();
    intMarker_.controls.push_back(control_);

    MarkerManager::instance().getInteractiveMarkerServer()->erase(intMarker_.name);
    MarkerManager::instance().getInteractiveMarkerServer()->applyChanges();
    MarkerManager::instance().getInteractiveMarkerServer()->insert(intMarker_, callback_);
    MarkerManager::instance().getInteractiveMarkerServer()->applyChanges();
}