#include "pch.h"
#include "maps/gridmap.h"
#include "util/parameters.h"

GridMap::GridMap(const std::string &name, double resolution,
                 Eigen::Vector3d origin) {
    name_ = name;
    resolution_ = resolution;
    origin_ = origin;
    mapPublisher_ = Parameters::instance().nodeHandle_->advertise<grid_map_msgs::GridMap>(name_, 1, true);
    initGridMap(origin_, resolution_);
}

void GridMap::initGridMap(Eigen::Vector3d &origin, double resolution) {
    const grid_map::Length mapLength(5.0, 5.0);
    const grid_map::Position mapPosition(origin.x(), origin.y());
    map_.setGeometry(mapLength, resolution, mapPosition);
    map_.setFrameId("world");
    tmpMap_.setGeometry(mapLength, resolution, mapPosition);
    tmpMap_.setFrameId("world");
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).", map_.getLength().x(), map_.getLength().y(),
             map_.getSize()(0), map_.getSize()(1));
}

void GridMap::addLayer(const std::string &layerName) {
    map_.add(layerName, 0.0);
}

Eigen::MatrixX2d GridMap::getMapSamplePositions() {
    Eigen::MatrixX2d gridMapPositions;
    gridMapPositions.conservativeResize(map_.getSize().x() * map_.getSize().y(), Eigen::NoChange);

    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
        grid_map::Position position;
        map_.getPosition(*it, position);
        gridMapPositions.row(it.getLinearIndex()) = position;
    }
    return gridMapPositions;
}

Eigen::MatrixX2d GridMap::getSubMapSamplePositions() {
    Eigen::MatrixX2d gridMapPositions;
    grid_map::Index submapStartIndex(right_, bottom_);
    grid_map::Index submapBufferSize(left_ - right_, top_ - bottom_);

    for (grid_map::SubmapIterator iterator(map_, submapStartIndex, submapBufferSize); !iterator.isPastEnd(); ++iterator) {
        grid_map::Position position;
        map_.getPosition(*iterator, position);
        gridMapPositions.conservativeResize(gridMapPositions.rows() + 1, Eigen::NoChange);
        gridMapPositions.bottomLeftCorner(1, position.size()) = position.transpose();
    }
    return gridMapPositions;
}

Eigen::MatrixX2d GridMap::getCircleSamplePositions(const Eigen::Vector2d &center, double radius) {
    Eigen::MatrixX2d gridMapPositions;
    for (grid_map::CircleIterator iterator(map_, center, radius); !iterator.isPastEnd(); ++iterator) {
        grid_map::Position position;
        map_.getPosition(*iterator, position);
        gridMapPositions.conservativeResize(gridMapPositions.rows() + 1, Eigen::NoChange);
        gridMapPositions.bottomLeftCorner(1, position.size()) = position.transpose();
    }
    return gridMapPositions;
}

void GridMap::updateLayer(const std::string &layerName, const Vector &values) {
    if (!existsLayer(layerName)) {
        STREAM_DEBUG("GridMap: updateLayer() Layer name \"" << layerName << "\" for update does not exist.");
        return;
    }

    grid_map::Matrix &layerData = (map_)[layerName];
    if (layerData.size() != values.size()) {
        STREAM_DEBUG("GridMap: updateLayer() values vector size is unexpected: " << values.size() << " / "
                                                                                << layerData.size());
    }

    grid_map::GridMapIterator it(map_);
    grid_map::Index index;
    int i = 0;
    while (!it.isPastEnd() && i < values.size()) {
        index = *it;
        layerData(index.x(), index.y()) = (float) values[i];
        i++;
        ++it;
    }
}

void GridMap::updateSubMapLayer(const std::string &layerName, const Vector &values) {
    if (!existsLayer(layerName)) {
        STREAM_DEBUG("GridMap: updateLayer() Layer name \"" << layerName << "\" for update does not exist.");
        return;
    }

    grid_map::Matrix &layerData = (map_)[layerName];
    grid_map::Index submapStartIndex(right_, bottom_);
    grid_map::Index submapBufferSize(left_ - right_, top_ - bottom_);
    grid_map::SubmapIterator it(map_, submapStartIndex, submapBufferSize);
    grid_map::Index index;
    int i = 0;
    while (!it.isPastEnd() && i < values.size()) {
        index = *it;
        layerData(index.x(), index.y()) = (float) values[i];
        i++;
        ++it;
    }
}

void GridMap::updateLayer(const std::string &layerName, const Vector &values, const Eigen::Vector2d &center, double radius) {
    if (!existsLayer(layerName)) {
        STREAM_DEBUG("GridMap: updateLayer() Layer name \"" << layerName << "\" for update does not exist.");
        return;
    }

    grid_map::Matrix &layerData = (map_)[layerName];
    grid_map::CircleIterator it(map_, center, radius);
    grid_map::Index index;
    int i = 0;
    while (!it.isPastEnd() && i < values.size()) {
        index = *it;
        layerData(index.x(), index.y()) = (float) values[i];
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
    mapPublisher_.publish(message);
}

bool GridMap::isInside(const Eigen::Vector2d &position) {
    return map_.isInside(position);
}

double GridMap::atPosition(const std::string &layerName, const Eigen::Vector2d &position) {
    if (!existsLayer(layerName)) {
        return -DBL_MAX;
    }
    return map_.atPosition(layerName, position.topRows(2));
}

bool GridMap::existsLayer(const std::string &layerName) {
    std::vector<std::string> layers = map_.getLayers();
    return std::find(layers.begin(), layers.end(), layerName) != layers.end();
}

void GridMap::updateGridDimensionWithSlamMap(const std::shared_ptr<nav_msgs::OccupancyGrid>& slamMap) {
    double width = (double) slamMap->info.width * slamMap->info.resolution;
    double height = (double) slamMap->info.height * slamMap->info.resolution;
    double posX = slamMap->info.origin.position.x + width / 2.0;
    double posY = slamMap->info.origin.position.y + height / 2.0;
    int width_i = ceil(width + slamMap->info.resolution);
    int height_i = ceil(height + slamMap->info.resolution);

    // Get most left, top, bottom and right occupied cell of slam map
    int left = slamMap->info.width;
    int right = 0;
    int top = slamMap->info.height;
    int bottom = 0;
    for (int i = 0; i < slamMap->data.size(); i++) {
        if (slamMap->data[i] >= 0) {
            int x = i % slamMap->info.width;
            int y = i / slamMap->info.width;
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

    const grid_map::Length mapSize(width_i, height_i);
    const grid_map::Position mapPosition(posX, posY);
    tmpMap_.setGeometry(mapSize, resolution_, mapPosition);
    map_.move(mapPosition);
    map_.extendToInclude(tmpMap_);

    int mapWidth = map_.getSize()(0);
    int mapHeight = map_.getSize()(1);
    double ratio = slamMap->info.resolution / resolution_;
    left_ = mapWidth - left * ratio;
    right_ = mapWidth - right * ratio;
    top_ = mapHeight - top * ratio;
    bottom_ = mapHeight - bottom * ratio;
}
