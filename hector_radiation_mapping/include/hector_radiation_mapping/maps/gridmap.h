#ifndef RADIATION_MAPPING_MAP2D_H
#define RADIATION_MAPPING_MAP2D_H

/**
 * @brief The GridMap class
 * Provides methods to add layers to the grid map, to update the layers and to publish the grid map.
 */
class GridMap {
public:
    GridMap(const std::string &name, double resolution, Vector3d origin = Vector3d(0, 0, 0));

    /**
     * Publishes the gridMap.
     */
    void publish();

    /**
     * Adds a layer to the grid map with the given name.
     * @param layerName
     */
    void addLayer(const std::string &layerName);

    /**
     * Updates the layer with the given name with the given values.
     * @param layerName
     * @param values
     */
    void updateLayer(const std::string &layerName, const Vector &values);

    /**
     * Updates the layer with the given name with the given values. Only updates the values within the submap.
     * The submap is defined by the parameters left, right, top and bottom and is the smallest bounding box around the
     * current occupancy grid.
     * @param layerName
     * @param values
     */
    void updateSubMapLayer(const std::string &layerName, const Vector &values);

    /**
     * Updates the layer with the given name with the given values. Only updates the values within the circle with the
     * given center and radius.
     * @param layerName
     * @param values
     * @param center
     * @param radius
     */
    void updateLayer(const std::string &layerName, const Vector &values, const Eigen::Vector2d &center, double radius);

    /**
     * Updates the grid map with the given slam map. The slam map is used to update the submap of the grid map.
     * @param slamMap
     */
    void updateGridDimensionWithSlamMap(const std::shared_ptr<nav_msgs::OccupancyGrid> &slamMap);

    /**
     * Checks if the given position is inside the grid map.
     * @param position
     * @return true if the given position is inside the grid map, false otherwise
     */
    bool isInside(const Eigen::Vector2d &position);

    /**
     * Checks if the given layer exists.
     * @param layerName
     * @return true if the given layer exists, false otherwise
     */
    bool existsLayer(const std::string &layerName);

    /**
     * Returns the value of the given layer at the given position.
     * @param layer_name
     * @param position
     * @return the value of the given layer at the given position
     */
    double atPosition(const std::string &layer_name, const Eigen::Vector2d &position);

    /**
     * Get the resolution of the grid map.
     * @return The resolution of the grid map.
     */
    double getResolution() { return resolution_; }

    /**
     * Get the GridMap.
     * @return The GridMap.
     */
    grid_map::GridMap getGridMap() { return map_; };

    /**
     * Get the map sample positions.
     * @return The map sample positions as a matrix. The columns contain x and y coordinates.
     */
    Eigen::MatrixX2d getMapSamplePositions();

    /**
     * Get the submap sample positions.
     * @return The submap sample positions as a matrix. The columns contain x and y coordinates.
     */
    Eigen::MatrixX2d getSubMapSamplePositions();

    /**
     * Get the map sample positions within the given radius of the given position.
     * @param center
     * @param radius
     * @return The map sample positions within the given radius of the given position as a matrix. The columns contain
     * x and y coordinates.
     */
    Eigen::MatrixX2d getCircleSamplePositions(const Eigen::Vector2d &center, double radius);

    /**
     * Get the grid map mutex.
     * @return The grid map mutex.
     */
    std::mutex &getGridMapMutex() { return mapMutex_; }

private:
    /**
     * Initializes the grid map with the given origin and resolution.
     * @param origin
     * @param resolution
     */
    void initGridMap(Eigen::Vector3d &origin, double resolution);

    // Variables
    double resolution_;
    Eigen::Vector3d origin_;
    std::string name_;
    int left_;
    int right_;
    int top_;
    int bottom_;

    // Map
    grid_map::GridMap map_;
    grid_map::GridMap tmpMap_;
    std::mutex mapMutex_;

    // ROS
    ros::Publisher mapPublisher_;
};

#endif //RADIATION_MAPPING_MAP2D_H