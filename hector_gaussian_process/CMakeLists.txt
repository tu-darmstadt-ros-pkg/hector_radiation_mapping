cmake_minimum_required(VERSION 3.0.2)
project(hector_gaussian_process)

find_package(catkin REQUIRED COMPONENTS
        rospy
        hector_radiation_mapping_msgs
        )

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES radiation_sensor_emulator
        #  CATKIN_DEPENDS interactive_markers radiation_msgs interactive_markers visualization_msgs rospy
        #  DEPENDS system_lib
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

#catkin_install_python(PROGRAMS
#        scripts/gp.py
#        scripts/gp_node
#        scripts/sample.py
#        scripts/gp_manager.py
#        scripts/utils.py
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

#install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        )
