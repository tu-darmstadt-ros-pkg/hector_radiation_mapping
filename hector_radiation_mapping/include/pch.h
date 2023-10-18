#ifndef RADIATION_MAPPING_PCH_H
#define RADIATION_MAPPING_PCH_H

#include <cmath>
#include <condition_variable>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <sys/stat.h>
#include <sys/resource.h>
#include <time.h>
#include "thread"

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <ros/ros.h>
#include <rosbag/view.h>
#include "std_msgs/Float64.h"
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "visualization_msgs/Marker.h"

#include "util/clock.h"

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrix;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;
typedef Eigen::Vector2d Vector2d;
typedef Eigen::Vector3d Vector3d;

enum DIMENSION {
    DIM2 = 2, DIM3 = 3
};

#define STREAM(x) if(true){ROS_INFO_STREAM(x);}
#define STREAM_ERROR(x) if(true){ROS_ERROR_STREAM(x);}
#define STREAM_DEBUG(x) if(false){ROS_INFO_STREAM(x);}

#endif //RADIATION_MAPPING_PCH_H