#include "marker/marker_manager.h"
#include "util/dddynamic_reconfigure.h"
#include "models/model_manager.h"
#include "models/gpython/gpython.h"
#include "models/gpython/gpython2D.h"
#include "models/gpython/gpython3D.h"
#include "models/model_exporter.h"

ModelManager::ModelManager() {
    sysCmdSub_ = ros::Subscriber(
            Parameters::instance().nodeHandle_->subscribe("/syscommand", 10, &ModelManager::sysCmdCallback, this));
    resetService_ = ros::ServiceServer(
            Parameters::instance().nodeHandle_->advertiseService("resetModel", &ModelManager::resetServiceCallback,
                                                                 this));

    ModelExporter::instance();
    GPython::instance();
    GPython2D::instance();
    GPython3D::instance();
}

ModelManager &ModelManager::instance() {
    static ModelManager instance;
    return instance;
}

void ModelManager::resetModels() {
    STREAM("Model reset.");
    MarkerManager::instance().deleteAllMarkers();
    MarkerManager::instance().deleteAllInteractiveMarkers();
    DDDynamicReconfigure::instance().reset();
}

void ModelManager::shutDown() {
    GPython::instance().shutDown();
    GPython2D::instance().shutDown();
    GPython3D::instance().shutDown();
}

bool ModelManager::resetServiceCallback(hector_radiation_mapping_msgs::ResetService::Request &req,
                                        hector_radiation_mapping_msgs::ResetService::Response &res) {
    resetModels();
    return true;
}

void ModelManager::sysCmdCallback(const std_msgs::String::ConstPtr &msg) {
    if (msg->data == "reset") {
        resetModels();
    }
}
