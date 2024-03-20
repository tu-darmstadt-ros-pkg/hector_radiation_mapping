#ifndef HECTOR_RADIATION_MAPPING_EXPLORATION_MAP_H
#define HECTOR_RADIATION_MAPPING_EXPLORATION_MAP_H

#include "pch.h"
#include "maps/gridmap.h"

class ExplorationMap {
public:
    struct Location{
        double min_distance_;
        Vector2d position_;
        u_int id_;
    };

    enum TRAVERSEABILITY {
        OCCUPIED = 1,
        FREE = 0,
        UNKNOWN = -1,
        UNREACHABLE = 2
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

    void addSampleLocation(const Vector2d &pos);
    Vector2d getClosestLocation(const Vector2d &pos);

private:
    ExplorationMap();
    ExplorationMap(const ExplorationMap &) = delete;
    ExplorationMap &operator=(const ExplorationMap &) = delete;

    /**
     * Callback for the slam map.
     * @param grid_msg_ptr
     */
    void slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg_ptr);

    void updateLoop();
    void addLocation(const Vector2d& pos, grid_map::Matrix &locations, grid_map::Index &index);
    void removeLocation(grid_map::Matrix &locations, grid_map::Index &index);
    void removeLocation(u_int id, grid_map::Matrix &locations);
    void updateExplorationMap(std::vector<Vector2d> &new_sample_locations);

    std::string layer_name_locations_;
    std::string layer_name_traverseability_;
    std::shared_ptr<GridMap> grid_map_;
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map_;
    std::shared_ptr<ros::Subscriber> slam_map_subscriber_;

    std::map<u_int, Location> locations_map_;
    u_int id_counter_ = 0;

    std::thread update_thread_;
    std::mutex update_mutex_;
    std::mutex sample_mutex_;
    std::condition_variable update_condition_;

    std::vector<Vector2d> new_sample_locations_;
};


#endif //HECTOR_RADIATION_MAPPING_EXPLORATION_MAP_H
