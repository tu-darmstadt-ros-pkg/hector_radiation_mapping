cmake_minimum_required(VERSION 3.5)
project(hector_radiation_mapping)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        ddynamic_reconfigure
        geometry_msgs
        grid_map_msgs
        grid_map_ros
        nav_msgs
        roscpp
        rosbag
        std_msgs
        tf2_ros
        pcl_ros
        interactive_markers
        ros_babel_fish
        hector_radiation_mapping_msgs
        ceres_catkin
        move_base_lite_msgs
        )

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED)
#find_package(Ceres REQUIRED)
find_package(GSL REQUIRED)

catkin_package(CATKIN_DEPENDS
        ceres_catkin
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        include/service
        include/hector_radiation_mapping
        include/
        ${EIGEN3_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

set(CMAKE_SKIP_BUILD_RPATH TRUE)

add_executable(radiation_mapping_node
        src/custom_geotiff/geotiff_writer.cpp
        src/maps/gridmap.cpp
        src/maps/pointcloud3d.cpp
        src/marker/marker.cpp
        src/marker/marker_manager.cpp
        src/marker/marker_text.cpp
        src/marker/marker_sphere.cpp
        src/marker/marker_trajectory.cpp
        src/hector_radiation_mapping/radiation_mapping_node.cpp
        src/hector_radiation_mapping/sampleManager.cpp
        src/models/gaussian_process/gaussian_process.cpp
        src/models/gaussian_process/gaussian_process_2d.cpp
        src/models/gaussian_process/gaussian_process_3d.cpp
        src/models/model_manager.cpp
        src/models/model_exporter.cpp
        src/models/model.cpp
        src/hector_radiation_mapping/source_interactive.cpp
        src/hector_radiation_mapping/source.cpp
        src/util/util.cpp
        src/util/color.cpp
        src/util/dddynamic_reconfigure.cpp
        src/models/least_squares/least_squares.cpp
        src/models/field_propagation/field_propagation.cpp
        src/models/baysian_inference/baysian_inference.cpp
        src/marker/marker_arrow.cpp
        src/models/triangulation/triangulation.cpp
        src/models/triangulation/tri_rsd.cpp
        include/hector_radiation_mapping/util/test.h
        src/exploration/exploration_manager.cpp
        src/exploration/exploration_map.cpp
        src/exploration/exploration_service.cpp
)

target_precompile_headers(radiation_mapping_node PRIVATE include/pch.h)

target_link_libraries(radiation_mapping_node
        ${catkin_LIBRARIES}
        Qt5::Widgets
        Qt5::Gui
        ${CERES_LIBRARIES}
        glog
        GSL::gsl
        GSL::gslcblas
        )

set_property(TARGET radiation_mapping_node PROPERTY CXX_STANDARD 17)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/)
