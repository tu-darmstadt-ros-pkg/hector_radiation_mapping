#include "exploration/exploration_map.h"
#include "util/parameters.h"
#include "hector_radiation_mapping/sampleManager.h"

ExplorationMap::ExplorationMap() {
    layer_name_sensor_locations_ = "sensor_locations";
    layer_name_base_locations_ = "base_locations";
    layer_name_traverseability_ = "traverseability";
    grid_map_ = std::make_shared<GridMap>(Parameters::instance().exploration_grid_map_topic,
                                          Parameters::instance().exploration_grid_map_resolution);
    grid_map_->addLayer(layer_name_sensor_locations_);
    grid_map_->addLayer(layer_name_base_locations_);
    grid_map_->addLayer(layer_name_traverseability_);

    slam_map_subscriber_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe<nav_msgs::OccupancyGrid>(
                    Parameters::instance().environment_map_topic, 1, &ExplorationMap::slamMapCallback, this));
}

void ExplorationMap::shutdown() {

}

void ExplorationMap::reset() {
    grid_map_->reset();
    slam_map_ = nullptr;
    sensor_locations_.clear();
    id_counter_ = 0;
    grid_map_->publish();
}

void ExplorationMap::updateExplorationMap() {
    if (slam_map_ == nullptr) {
        return;
    }

    std::vector<Vector2d> new_sample_locations;
    std::vector<Vector2d> new_base_locations;
    {
        std::lock_guard<std::mutex> lock(sample_mutex_);
        new_sample_locations = new_sensor_locations_;
        new_base_locations = new_base_locations_;
        new_sensor_locations_.clear();
        new_base_locations_.clear();
    }
    Clock clock;
    clock.tick();

    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map = slam_map_;
    grid_map_->updateGridDimensionWithSlamMap(slam_map);
    grid_map::GridMap &grid_map_ref = grid_map_->getGridMapRef();
    grid_map::Matrix &sensor_locations = (grid_map_ref)[layer_name_sensor_locations_];
    grid_map::Matrix &base_locations = (grid_map_ref)[layer_name_base_locations_];

    grid_map::Matrix &traverseability = (grid_map_ref)[layer_name_traverseability_];
    grid_map::Matrix old_layer = traverseability;
    traverseability.setConstant(TRAVERSEABILITY::NONE);

    int grid_map_width = grid_map_ref.getSize()(0);
    int grid_map_height = grid_map_ref.getSize()(1);
    double slam_map_resolution = slam_map->info.resolution;
    Vector2d slam_origin(slam_map->info.origin.position.x, slam_map->info.origin.position.y); //bottom left corner

    // update traverseability map
    for (int i = 0; i < slam_map->data.size(); i++) {
        if (slam_map->data[i] == 0) {
            int x = i % slam_map->info.width;
            int y = i / slam_map->info.width;

            // get position
            Vector2d pos(slam_origin.x() + x * slam_map_resolution, slam_origin.y() + y * slam_map_resolution);

            grid_map::Index index;
            if (!grid_map_ref.getIndex(pos, index)) {
                continue;
            }
            if (index(0) < 0 || index(0) >= grid_map_width || index(1) < 0 || index(1) >= grid_map_height) {
                continue;
            }

            traverseability(index(0), index(1)) = TRAVERSEABILITY::FREE;
        }
    }

    for (int i = 0; i < slam_map->data.size(); i++) {
        if (slam_map->data[i] != 0 && slam_map->data[i] != 100) {
            int x = i % slam_map->info.width;
            int y = i / slam_map->info.width;

            // get position
            Vector2d pos(slam_origin.x() + x * slam_map_resolution, slam_origin.y() + y * slam_map_resolution);

            grid_map::Index index;
            if (!grid_map_ref.getIndex(pos, index)) {
                continue;
            }
            if (index(0) < 0 || index(0) >= grid_map_width || index(1) < 0 || index(1) >= grid_map_height) {
                continue;
            }

            traverseability(index(0), index(1)) = TRAVERSEABILITY::UNKNOWN;
        }
    }

    for (int i = 0; i < slam_map->data.size(); i++) {
        if (slam_map->data[i] == 100) {
            int x = i % slam_map->info.width;
            int y = i / slam_map->info.width;

            // get position
            Vector2d pos(slam_origin.x() + x * slam_map_resolution, slam_origin.y() + y * slam_map_resolution);

            grid_map::Index index;
            if (!grid_map_ref.getIndex(pos, index)) {
                continue;
            }
            if (index(0) < 0 || index(0) >= grid_map_width || index(1) < 0 || index(1) >= grid_map_height) {
                continue;
            }

            traverseability(index(0), index(1)) = TRAVERSEABILITY::OCCUPIED;
        }
    }

    std::lock_guard<std::mutex> lock(locations_mutex_);
    // update sensor_locations
    for (grid_map::GridMapIterator it(grid_map_ref); !it.isPastEnd(); ++it) {
        Vector2d position;
        grid_map_ref.getPosition(*it, position);
        grid_map::Index index = *it;
        int row = index(0);
        int col = index(1);

        // if was explored/unreachable -> keep it
        if (old_layer(row, col) == TRAVERSEABILITY::EXPLORED) {
            traverseability(row, col) = TRAVERSEABILITY::EXPLORED;
        }
        if (old_layer(row, col) == TRAVERSEABILITY::UNREACHABLE) {
            traverseability(row, col) = TRAVERSEABILITY::UNREACHABLE;
        }
        // if still none -> set to unknown
        if (traverseability(row, col) == TRAVERSEABILITY::NONE) {
            traverseability(row, col) = TRAVERSEABILITY::UNKNOWN;
        }

        // if was free and now is occupied -> remove location
        if (traverseability(row, col) == TRAVERSEABILITY::OCCUPIED &&
            old_layer(row, col) == TRAVERSEABILITY::FREE &&
            sensor_locations(row, col) != TRAVERSEABILITY::UNKNOWN) {
            removeLocation(sensor_locations, base_locations, index);
        }

        // if was unknown or occupied and now is free or was missing -> add location
        if (traverseability(row, col) == TRAVERSEABILITY::FREE &&
            (old_layer(row, col) == TRAVERSEABILITY::UNKNOWN ||
             old_layer(row, col) == TRAVERSEABILITY::OCCUPIED ||
             sensor_locations(row, col) == TRAVERSEABILITY::UNKNOWN ||
             isnan(sensor_locations(row, col)))) {

            // sort out edges
            // check if any of the neighbors is OCCUPIED
            bool neighbor_occupied = false;
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    if (i == 0 && j == 0) {
                        continue;
                    }
                    if (row + i < 0 || row + i >= grid_map_width || col + j < 0 || col + j >= grid_map_height) {
                        continue;
                    }
                    if (traverseability(row + i, col + j) == TRAVERSEABILITY::OCCUPIED) {
                        neighbor_occupied = true;
                        break;
                    }
                }
                if (neighbor_occupied) {
                    break;
                }
            }

            addLocation(position, sensor_locations, base_locations, index, !neighbor_occupied);
        }
    }

    // update min distance to samples for all sensor_locations
    for (Vector2d &new_sample_location : new_sample_locations) {
        for (auto &location : sensor_locations_) {
            location.second.min_distance_ = std::min(location.second.min_distance_, (location.second.position_ - new_sample_location).norm());
            if (location.second.min_distance_ < 0.5) {
                location.second.explored_ = true;
                grid_map::Index index;
                grid_map_ref.getIndex(location.second.position_, index);
                traverseability(index(0), index(1)) = TRAVERSEABILITY::EXPLORED;
            }
        }
    }
    grid_map_->publish();

    STREAM("ExplorationMap update took: " << clock.tock() << " ms");
}

void ExplorationMap::addLocation(const Vector2d& pos, grid_map::Matrix &sensor_locations, grid_map::Matrix &base_locations, grid_map::Index &index, bool base) {
    SensorLocation sensor_location;
    BaseLocation base_location;
    sensor_location.position_ = pos;
    base_location.position_ = pos;
    sensor_location.id_ = id_counter_;
    base_location.id_ = id_counter_;
    id_counter_++;
    sensor_location.min_distance_ = SampleManager::instance().getMinDistanceToAllSamples(pos);
    sensor_locations_[sensor_location.id_] = sensor_location;
    base_locations_[base_location.id_] = base_location;
    sensor_locations(index(0), index(1)) = (float) sensor_location.id_;
    if (base) {
        base_locations(index(0), index(1)) = (float) base_location.id_;
    }
}

void ExplorationMap::removeLocation(grid_map::Matrix &sensor_locations, grid_map::Matrix &base_locations, grid_map::Index &index) {
    auto id = (u_int) sensor_locations(index(0), index(1));
    sensor_locations_.erase(id);
    base_locations_.erase(id);
    sensor_locations(index(0), index(1)) = TRAVERSEABILITY::UNKNOWN;
    base_locations(index(0), index(1)) = TRAVERSEABILITY::UNKNOWN;
}

void ExplorationMap::slamMapCallback(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &grid_msg_ptr) {
    slam_map_ = std::make_shared<nav_msgs::OccupancyGrid>(*grid_msg_ptr);
}

void ExplorationMap::addSampleLocation(const Vector2d &sensor_position, const Vector2d &base_position) {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    new_sensor_locations_.push_back(sensor_position);
    new_base_locations_.push_back(base_position);
}

bool ExplorationMap::getClosestSensorLocation(const Vector2d &pos, Vector2d &closest_location) {
    std::lock_guard<std::mutex> lock(locations_mutex_);
    double min_distance = std::numeric_limits<double>::max();
    for (auto &location : sensor_locations_) {
        if (!location.second.reachable_ || location.second.explored_) {
            continue;
        }

        double distance = (location.second.position_ - pos).norm();
        if (distance < min_distance) {
            min_distance = distance;
            closest_location = location.second.position_;
        }
    }

    return min_distance != std::numeric_limits<double>::max();
}

bool ExplorationMap::getClosestBaseLocation(const Vector2d &pos, Vector2d &closest_location) {
    std::lock_guard<std::mutex> lock(locations_mutex_);
    double min_distance = std::numeric_limits<double>::max();
    for (auto &location : base_locations_) {
        double distance = (location.second.position_ - pos).norm();
        if (distance < min_distance) {
            min_distance = distance;
            closest_location = location.second.position_;
        }
    }

    return min_distance != std::numeric_limits<double>::max();
}

void ExplorationMap::setLocationExplored(Vector2d &pos, bool explored) {
    grid_map::GridMap &grid_map_ref = grid_map_->getGridMapRef();
    grid_map::Matrix &locations = (grid_map_ref)[layer_name_sensor_locations_];
    grid_map::Matrix &traverseability = (grid_map_ref)[layer_name_traverseability_];

    grid_map::Index index;
    grid_map_ref.getIndex(pos, index);
    auto id = (u_int) locations(index(0), index(1));
    if (sensor_locations_.find(id) != sensor_locations_.end()) {
        sensor_locations_[id].explored_ = explored;
    }

    if (explored) {
        traverseability(index(0), index(1)) = TRAVERSEABILITY::EXPLORED;
    }
}

void ExplorationMap::setLocationReachable(Vector2d &pos, bool reachable) {
    grid_map::GridMap &grid_map_ref = grid_map_->getGridMapRef();
    grid_map::Matrix &locations = (grid_map_ref)[layer_name_sensor_locations_];
    grid_map::Matrix &traverseability = (grid_map_ref)[layer_name_traverseability_];

    grid_map::Index index;
    grid_map_ref.getIndex(pos, index);
    auto id = (u_int) locations(index(0), index(1));
    if (sensor_locations_.find(id) != sensor_locations_.end()) {
        sensor_locations_[id].reachable_ = reachable;
    }

    if (reachable) {
        traverseability(index(0), index(1)) = TRAVERSEABILITY::UNREACHABLE;
    }
}