#ifndef HECTOR_RADIATION_MAPPING_EXPLORATION_MAP_H
#define HECTOR_RADIATION_MAPPING_EXPLORATION_MAP_H

#include "pch.h"
#include "maps/gridmap.h"

class ExplorationMap {
public:
    struct SensorLocation{
        Vector2d position_;
        u_int id_;
        double min_distance_ = DBL_MAX;
        bool explored_ = false;
        bool reachable_ = true;
    };

    struct BaseLocation{
        Vector2d position_;
        u_int id_;
    };

    enum TRAVERSEABILITY {
        OCCUPIED = 0,
        FREE = 1,
        EXPLORED = 2,
        NONE = -1,
        UNKNOWN = -2,
        UNREACHABLE = -3
    };

    /**
     * Returns the class instance.
     * @return class instance.
     */
    static ExplorationMap &instance() {
        static ExplorationMap instance;
        return instance;
    }

    void reset();
    void shutdown();

    void addSampleLocation(const Vector2d &sensor_position, const Vector2d &base_position);

    bool getClosestSensorLocation(const Vector2d &pos, Vector2d &closest_location);
    bool getClosestBaseLocation(const Vector2d &pos, Vector2d &closest_location);

    bool hasLocations() {
        return !sensor_locations_.empty();
    }

    void setLocationExplored(Vector2d &pos, bool explored);
    void setLocationReachable(Vector2d &pos, bool reachable);
    void updateExplorationMap();

private:
    ExplorationMap();
    ExplorationMap(const ExplorationMap &) = delete;
    ExplorationMap &operator=(const ExplorationMap &) = delete;

    /**
     * Callback for the slam map.
     * @param grid_msg_ptr
     */
    void slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg_ptr);

    void addLocation(const Vector2d& pos, grid_map::Matrix &sensor_locations, grid_map::Matrix &base_locations, grid_map::Index &index, bool base);
    void removeLocation(grid_map::Matrix &sensor_locations, grid_map::Matrix &base_locations, grid_map::Index &index);

    std::string layer_name_sensor_locations_;
    std::string layer_name_base_locations_;
    std::string layer_name_traverseability_;
    std::shared_ptr<GridMap> grid_map_;
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map_;
    std::shared_ptr<ros::Subscriber> slam_map_subscriber_;

    std::map<u_int, SensorLocation> sensor_locations_;
    std::map<u_int, BaseLocation> base_locations_;
    std::mutex locations_mutex_;
    u_int id_counter_ = 0;

    std::mutex sample_mutex_;
    std::vector<Vector2d> new_sensor_locations_;
    std::vector<Vector2d> new_base_locations_;
};

#endif //HECTOR_RADIATION_MAPPING_EXPLORATION_MAP_H
