#include "pch.h"
#include "hector_radiation_mapping/radiation_mapping_node.h"
#include "hector_radiation_mapping/sampleManager.h"
#include "models/model_manager.h"
#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "marker/marker_manager.h"

RadiationMapper::RadiationMapper() {
    // Initialize Subsystems
    DDDynamicReconfigure::instance();
    MarkerManager::instance();
    SampleManager::instance();
    ModelManager::instance();

    STREAM("hector_radiation_mapping node initialized!");
    ros::MultiThreadedSpinner spinner(Parameters::instance().rosSpinnerThreads);
    spinner.spin();
}

void RadiationMapper::sigintHandler(int sig) {
    // shutdown procedure
    STREAM("hector_radiation_mapping shutdown procedure...");
    MarkerManager::instance().deleteAllMarkers();
    MarkerManager::instance().deleteAllInteractiveMarkers();
    ModelManager::instance().shutDown();

    DDDynamicReconfigure::instance().reset();
    ros::shutdown();
}

int main(int argc, char **argv) {
    // Initialize the ROS environment
    ros::init(argc, argv, "hector_radiation_mapping", ros::init_options::NoSigintHandler);
    signal(SIGINT, &RadiationMapper::sigintHandler);
    RadiationMapper radiationMapper;

    STREAM("hector_radiation_mapping node finished!");
    return 0;
}
