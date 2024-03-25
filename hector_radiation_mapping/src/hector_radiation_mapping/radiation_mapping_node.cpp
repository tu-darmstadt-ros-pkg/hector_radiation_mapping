#include "pch.h"
#include "hector_radiation_mapping/radiation_mapping_node.h"
#include "hector_radiation_mapping/sampleManager.h"
#include "models/model_manager.h"
#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "marker/marker_manager.h"
#include "exploration/exploration_manager.h"

RadiationMapper::RadiationMapper() {
    sys_cmd_sub_ = ros::Subscriber(
            Parameters::instance().node_handle_ptr_->subscribe("/syscommand", 10, &RadiationMapper::sysCmdCallback));
    reset_service_ = ros::ServiceServer(
            Parameters::instance().node_handle_ptr_->advertiseService("resetModel", &RadiationMapper::resetServiceCallback));

    // Initialize Subsystems
    DDDynamicReconfigure::instance();
    MarkerManager::instance();
    SampleManager::instance();
    ModelManager::instance();
    ExplorationManager::instance();

    STREAM("hector_radiation_mapping node initialized!");
    ros::MultiThreadedSpinner spinner(Parameters::instance().ros_spinner_threads);
    spinner.spin();
}

void RadiationMapper::sigintHandler(int sig) {
    // shutdown procedure
    STREAM("hector_radiation_mapping shutdown procedure...");
    MarkerManager::instance().deleteAllMarkers();
    MarkerManager::instance().deleteAllInteractiveMarkers();
    ModelManager::instance().shutDown();
    ExplorationManager::instance().shutdown();
    DDDynamicReconfigure::instance().reset();
    ros::shutdown();
}

bool RadiationMapper::resetServiceCallback(hector_radiation_mapping_msgs::ResetService::Request &req,
                                           hector_radiation_mapping_msgs::ResetService::Response &res) {
    reset();
    return true;
}

void RadiationMapper::sysCmdCallback(const std_msgs::String_<std::allocator<void>>::ConstPtr &msg) {
    if (msg->data == "reset") {
        reset();
    }
}

void RadiationMapper::reset() {
    ModelManager::instance().resetModels();
    SampleManager::instance().reset();
    MarkerManager::instance().deleteAllMarkers();
    MarkerManager::instance().deleteAllInteractiveMarkers();
    ExplorationManager::instance().reset();
}

int main(int argc, char **argv) {
    // Initialize the ROS environment
    ros::init(argc, argv, "hector_radiation_mapping", ros::init_options::NoSigintHandler);
    signal(SIGINT, &RadiationMapper::sigintHandler);
    RadiationMapper radiationMapper;

    STREAM("hector_radiation_mapping node finished!");
    return 0;
}
