#include "exploration/exploration_map.h"
#include "util/parameters.h"
#include "hector_radiation_mapping/sampleManager.h"

ExplorationMap::ExplorationMap() {
    layer_name_locations_ = "locations";
    layer_name_traverseability_ = "traverseability";
    grid_map_ = std::make_shared<GridMap>(Parameters::instance().exploration_grid_map_topic,
                                          Parameters::instance().exploration_grid_map_resolution);
    grid_map_->addLayer(layer_name_locations_);
    grid_map_->addLayer(layer_name_traverseability_);

    slam_map_subscriber_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe<nav_msgs::OccupancyGrid>(
                    Parameters::instance().environment_map_topic, 1, &ExplorationMap::slamMapCallback, this));

    update_thread_ = std::thread(&ExplorationMap::updateLoop, this);
}

void ExplorationMap::reset() {
    grid_map_->reset();
    slam_map_ = nullptr;
    locations_map_.clear();
    id_counter_ = 0;
    grid_map_->publish();
}

void ExplorationMap::updateLoop() {
    std::vector<Vector2d> new_sample_locations;
    while (ros::ok()) {
        std::unique_lock<std::mutex> lock(update_mutex_);
        update_condition_.wait(lock, [this] { return !new_sample_locations_.empty(); });
        {
            std::lock_guard<std::mutex> lock2(sample_mutex_);
            new_sample_locations = new_sample_locations_;
            new_sample_locations_.clear();
        }
        updateExplorationMap(new_sample_locations);
    }

    STREAM("ExplorationMap update loop ended.");
}

void ExplorationMap::updateExplorationMap(std::vector<Vector2d> &new_sample_locations) {
    if (slam_map_ == nullptr) {
        return;
    }
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map = slam_map_;
    std::lock_guard<std::mutex> lock(update_mutex_);
    grid_map_->updateGridDimensionWithSlamMap(slam_map);
    grid_map::GridMap &grid_map_ref = grid_map_->getGridMapRef();
    grid_map::Matrix &locations = (grid_map_ref)[layer_name_locations_];

    grid_map::Matrix &traverseability = (grid_map_ref)[layer_name_traverseability_];
    grid_map::Matrix old_layer = traverseability;
    traverseability.setConstant(TRAVERSEABILITY::UNKNOWN);

    int grid_map_width = grid_map_ref.getSize()(0);
    int grid_map_height = grid_map_ref.getSize()(1);
    double slam_map_resolution = slam_map->info.resolution;
    Vector2d slam_origin(slam_map->info.origin.position.x, slam_map->info.origin.position.y); //bottom left corner

    // update traverseability map
    for (int i = 0; i < slam_map->data.size(); i++) {
        if (slam_map->data[i] >= 0) {
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

            if (slam_map->data[i] == 0 && traverseability(index(0), index(1)) == TRAVERSEABILITY::UNKNOWN) {
                traverseability(index(0), index(1)) = TRAVERSEABILITY::FREE;
            } else if (slam_map->data[i] > 0) {
                traverseability(index(0), index(1)) = TRAVERSEABILITY::OCCUPIED;
            }
        }
    }

    // update locations
    for (grid_map::GridMapIterator it(grid_map_ref); !it.isPastEnd(); ++it) {
        Vector2d position;
        grid_map_ref.getPosition(*it, position);
        grid_map::Index index = *it;
        int row = index(0);
        int col = index(1);

        // if was unknown or occupied and now is free or was missing -> add location
        if (traverseability(row, col) == TRAVERSEABILITY::FREE &&
            (old_layer(row, col) == TRAVERSEABILITY::UNKNOWN ||
             old_layer(row, col) == TRAVERSEABILITY::OCCUPIED ||
             locations(row, col) == TRAVERSEABILITY::UNKNOWN ||
             isnan(locations(row, col)))) {
            addLocation(position, locations, index);
        }

        // if was free and now is occupied -> remove location
        if (traverseability(row, col) == TRAVERSEABILITY::OCCUPIED &&
            old_layer(row, col) == TRAVERSEABILITY::FREE &&
            locations(row, col) != TRAVERSEABILITY::UNKNOWN) {
            removeLocation(locations, index);
        }
    }

    // update min distance to samples for all locations
    for (auto &location : locations_map_) {
        for (Vector2d &new_sample_location : new_sample_locations) {
            location.second.min_distance_ = std::min(location.second.min_distance_, (location.second.position_ - new_sample_location).norm());
        }
    }
    grid_map_->publish();
}

void ExplorationMap::addLocation(const Vector2d& pos, grid_map::Matrix &locations, grid_map::Index &index) {
    Location location;
    location.position_ = pos;
    location.id_ = id_counter_;
    id_counter_++;
    location.min_distance_ = SampleManager::instance().getMinDistanceToAllSamples(pos);
    locations_map_[location.id_] = location;
    locations(index(0), index(1)) = (float) location.id_;
}

void ExplorationMap::removeLocation(grid_map::Matrix &locations, grid_map::Index &index) {
    auto id = (u_int) locations(index(0), index(1));
    locations_map_.erase(id);
    locations(index(0), index(1)) = TRAVERSEABILITY::UNKNOWN;
}

void ExplorationMap::removeLocation(u_int id, grid_map::Matrix &locations) {
    grid_map::Index index;
    grid_map_->getGridMapRef().getIndex(locations_map_[id].position_, index);
    locations_map_.erase(id);
    locations(index(0), index(1)) = TRAVERSEABILITY::UNKNOWN;
}

void ExplorationMap::slamMapCallback(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &grid_msg_ptr) {
    slam_map_ = std::make_shared<nav_msgs::OccupancyGrid>(*grid_msg_ptr);
}

void ExplorationMap::addSampleLocation(const Vector2d &pos) {
    {
        std::lock_guard<std::mutex> lock(sample_mutex_);
        new_sample_locations_.push_back(pos);
    }
    update_condition_.notify_one();
}

Vector2d ExplorationMap::getClosestLocation(const Vector2d &pos) {
    double min_distance = std::numeric_limits<double>::max();
    Vector2d closest_location;
    for (auto &location : locations_map_) {
        double distance = (location.second.position_ - pos).norm();
        if (distance < min_distance) {
            min_distance = distance;
            closest_location = location.second.position_;
        }
    }
    return closest_location;
}
