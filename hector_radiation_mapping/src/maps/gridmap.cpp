#include "pch.h"
#include "maps/gridmap.h"
#include "util/parameters.h"

GridMap::GridMap(const std::string &name, double resolution,
                 Eigen::Vector3d origin) {
    name_ = name;
    resolution_ = resolution;
    origin_ = origin;
    map_publisher_ = Parameters::instance().node_handle_ptr_->advertise<grid_map_msgs::GridMap>(name_, 1, true);
    initGridMap(origin_, resolution_);
    ROS_INFO("GridMap with topic \"%s\" created.", name_.c_str());
    //ROS_INFO("Created map with size %f x %f m (%i x %i cells).", map_.getLength().x(), map_.getLength().y(),
    //         map_.getSize()(0), map_.getSize()(1));
}

void GridMap::initGridMap(Eigen::Vector3d &origin, double resolution) {
    const grid_map::Length mapLength(5.0, 5.0);
    const grid_map::Position mapPosition(origin.x(), origin.y());
    map_.setGeometry(mapLength, resolution, mapPosition);
    map_.setFrameId(Parameters::instance().world_frame);
    tmp_map_.setGeometry(mapLength, resolution, mapPosition);
    tmp_map_.setFrameId(Parameters::instance().world_frame);
}

void GridMap::addLayer(const std::string &layer_name) {
    if (existsLayer(layer_name)) {
        STREAM_DEBUG("GridMap: addLayer() Layer name \"" << layer_name << "\" already exists.");
        return;
    }
    map_.add(layer_name, 0.0);
}

Matrix GridMap::getSamplePositions(const std::shared_ptr<nav_msgs::OccupancyGrid> &slam_map, bool use_circle, double radius,
                            const Vector2d &center) {
    if (slam_map != nullptr) {
        updateGridDimensionWithSlamMap(slam_map);
    }
    return use_circle ? getCircleSamplePositions(center, radius)
                      : getMapSamplePositions();
}

Eigen::MatrixX2d GridMap::getMapSamplePositions() {
    Eigen::MatrixX2d grid_map_positions;
    grid_map_positions.conservativeResize(map_.getSize().x() * map_.getSize().y(), Eigen::NoChange);

    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
        grid_map::Position position;
        map_.getPosition(*it, position);
        grid_map_positions.row(it.getLinearIndex()) = position;
    }
    return grid_map_positions;
}

Eigen::MatrixX2d GridMap::getSubMapSamplePositions() {
    Eigen::MatrixX2d grid_map_positions;
    grid_map::Index sub_map_start_index(right_, bottom_);
    grid_map::Index sub_map_buffer_size(left_ - right_, top_ - bottom_);

    for (grid_map::SubmapIterator iterator(map_, sub_map_start_index,
                                           sub_map_buffer_size); !iterator.isPastEnd(); ++iterator) {
        grid_map::Position position;
        map_.getPosition(*iterator, position);
        grid_map_positions.conservativeResize(grid_map_positions.rows() + 1, Eigen::NoChange);
        grid_map_positions.bottomLeftCorner(1, position.size()) = position.transpose();
    }
    return grid_map_positions;
}

Eigen::MatrixX2d GridMap::getCircleSamplePositions(const Eigen::Vector2d &center, double radius) {
    Eigen::MatrixX2d grid_map_positions;
    for (grid_map::CircleIterator iterator(map_, center, radius); !iterator.isPastEnd(); ++iterator) {
        grid_map::Position position;
        map_.getPosition(*iterator, position);
        grid_map_positions.conservativeResize(grid_map_positions.rows() + 1, Eigen::NoChange);
        grid_map_positions.bottomLeftCorner(1, position.size()) = position.transpose();
    }
    return grid_map_positions;
}

void GridMap::updateLayer(const std::string &layer_name, const Vector &values) {
    if (!existsLayer(layer_name)) {
        STREAM_DEBUG("GridMap: updateLayer() Layer name \"" << layer_name << "\" for update does not exist.");
        return;
    }

    grid_map::Matrix &layer_data = (map_)[layer_name];
    if (layer_data.size() != values.size()) {
        STREAM_DEBUG("GridMap: updateLayer() values vector size is unexpected: " << values.size() << " / "
                                                                                 << layer_data.size());
    }

    grid_map::GridMapIterator it(map_);
    grid_map::Index index;
    int i = 0;
    while (!it.isPastEnd() && i < values.size()) {
        index = *it;
        layer_data(index.x(), index.y()) = (float) values[i];
        i++;
        ++it;
    }
}

void GridMap::updateSubMapLayer(const std::string &layer_name, const Vector &values) {
    if (!existsLayer(layer_name)) {
        STREAM_DEBUG("GridMap: updateLayer() Layer name \"" << layer_name << "\" for update does not exist.");
        return;
    }

    grid_map::Matrix &layer_data = (map_)[layer_name];
    grid_map::Index sub_map_start_index(right_, bottom_);
    grid_map::Index sub_map_buffer_size(left_ - right_, top_ - bottom_);
    grid_map::SubmapIterator it(map_, sub_map_start_index, sub_map_buffer_size);
    grid_map::Index index;
    int i = 0;
    while (!it.isPastEnd() && i < values.size()) {
        index = *it;
        layer_data(index.x(), index.y()) = (float) values[i];
        i++;
        ++it;
    }
}

void
GridMap::updateLayer(const std::string &layer_name, const Vector &values, const Eigen::Vector2d &center,
                     double radius) {
    if (!existsLayer(layer_name)) {
        STREAM_DEBUG("GridMap: updateLayer() Layer name \"" << layer_name << "\" for update does not exist.");
        return;
    }

    grid_map::Matrix &layer_data = (map_)[layer_name];
    grid_map::CircleIterator it(map_, center, radius);
    grid_map::Index index;
    int i = 0;
    while (!it.isPastEnd() && i < values.size()) {
        index = *it;
        layer_data(index.x(), index.y()) = (float) values[i];
        i++;
        ++it;
    }
    if (!it.isPastEnd()) {
        STREAM_DEBUG("GridMap: updateLayer() values vector had less values than circleIterator");
    } else if (i < values.size()) {
        STREAM_DEBUG("GridMap: updateLayer() values vector had more values than circleIterator");
    }
}

void GridMap::publish() {
    map_.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map_, message);
    map_publisher_.publish(message);
}

bool GridMap::isInside(const Eigen::Vector2d &position) {
    return map_.isInside(position);
}

double GridMap::atPosition(const std::string &layer_name, const Eigen::Vector2d &position) {
    if (!existsLayer(layer_name)) {
        return -DBL_MAX;
    }
    return map_.atPosition(layer_name, position.topRows(2));
}

bool GridMap::existsLayer(const std::string &layer_name) {
    std::vector<std::string> layers = map_.getLayers();
    return std::find(layers.begin(), layers.end(), layer_name) != layers.end();
}

void GridMap::updateGridDimensionWithSlamMap(const std::shared_ptr<nav_msgs::OccupancyGrid> &slam_map) {
    double width = (double) slam_map->info.width * slam_map->info.resolution;
    double height = (double) slam_map->info.height * slam_map->info.resolution;
    double posX = slam_map->info.origin.position.x + width / 2.0;
    double posY = slam_map->info.origin.position.y + height / 2.0;
    int width_i = ceil(width + slam_map->info.resolution);
    int height_i = ceil(height + slam_map->info.resolution);

    // Get most left, top, bottom and right occupied cell of slam map
    int left = slam_map->info.width;
    int right = 0;
    int top = slam_map->info.height;
    int bottom = 0;
    for (int i = 0; i < slam_map->data.size(); i++) {
        if (slam_map->data[i] >= 0) {
            int x = i % slam_map->info.width;
            int y = i / slam_map->info.width;
            if (x < left) {
                left = x;
            }
            if (x > right) {
                right = x;
            }
            if (y < top) {
                top = y;
            }
            if (y > bottom) {
                bottom = y;
            }
        }
    }

    const grid_map::Length map_size(width_i, height_i);
    const grid_map::Position map_position(posX, posY);
    tmp_map_.setGeometry(map_size, resolution_, map_position);
    map_.move(map_position);
    map_.extendToInclude(tmp_map_);

    int map_width = map_.getSize()(0);
    int map_height = map_.getSize()(1);
    double ratio = slam_map->info.resolution / resolution_;
    left_ = map_width - left * ratio;
    right_ = map_width - right * ratio;
    top_ = map_height - top * ratio;
    bottom_ = map_height - bottom * ratio;
}
