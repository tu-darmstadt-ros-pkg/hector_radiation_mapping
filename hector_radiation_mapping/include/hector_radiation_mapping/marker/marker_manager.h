#ifndef RADIATION_MAPPING_MARKER_MANAGER_H
#define RADIATION_MAPPING_MARKER_MANAGER_H

#include <interactive_markers/interactive_marker_server.h>

class MarkerManager {
public:
    /**
     * Returns the instance of the MarkerManager class.
     * @return The instance of the MarkerManager class.
     */
    static MarkerManager &instance();

    /**
     * Deletes all markers.
     */
    void deleteAllMarkers();

    /**
     * Deletes the given marker.
     * @param marker
     */
    void deleteMarker(visualization_msgs::Marker &marker);

    /**
     * Deletes all interactive markers.
     */
    void deleteAllInteractiveMarkers();

    /**
     * Publishes the given marker.
     * @param marker
     */
    void publishMarker(visualization_msgs::Marker &marker);

    /**
     * Get the interactive marker server.
     * @return The interactive marker server.
     */
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> getInteractiveMarkerServer() {
        return interactiveMarkerServer_;
    };

private:
    MarkerManager();
    MarkerManager(const MarkerManager &) = delete;
    MarkerManager &operator=(const MarkerManager &) = delete;

    std::shared_ptr<ros::NodeHandle> nodeHandle_;
    std::shared_ptr<ros::Publisher> markerPublisher_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactiveMarkerServer_;
};

#endif //RADIATION_MAPPING_MARKER_MANAGER_H
