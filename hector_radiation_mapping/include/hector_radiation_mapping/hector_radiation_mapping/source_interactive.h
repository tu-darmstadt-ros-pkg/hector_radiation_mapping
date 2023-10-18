#ifndef RADIATION_MAPPING_SOURCE_INTERACTIVE_H
#define RADIATION_MAPPING_SOURCE_INTERACTIVE_H

#include <interactive_markers/interactive_marker_server.h>

#include "pch.h"
#include "source.h"


/**
 * @brief The SourceInteractive class
 * This class represents a source of radiation. It contains the position, the cps and the dose rate of the source.
 * It also contains a marker that is used to visualize the source in rviz.
 * This class is used for interactive sources, which can be confirmed by clicking on them in rviz.
 */
class SourceInteractive : public Source {
public:
    SourceInteractive(const Eigen::Vector3d &position, double cps, double doseRate, interactive_markers::InteractiveMarkerServer::FeedbackCallback callback);
    ~SourceInteractive();

    /**
     * Set whether the source is confirmed or not
     * @param confirmed
     */
    void setConfirmed(bool confirmed) override;

    /**
     * Check if the source was confirmed. This is used to check if a new location for the evaluation of the Model
     * should be added.
     * @return true if the source was confirmed
     */
    bool wasConfirmed() const {return wasConfirmed_;};

private:
    bool wasConfirmed_;
    visualization_msgs::InteractiveMarker intMarker_;
    visualization_msgs::Marker visualMarker_;
    visualization_msgs::InteractiveMarkerControl control_;
    interactive_markers::InteractiveMarkerServer::FeedbackCallback callback_;
};

#endif //RADIATION_MAPPING_SOURCE_INTERACTIVE_H
