#include "marker/marker_manager.h"
#include "util/dddynamic_reconfigure.h"
#include "models/model_manager.h"
#include "models/gpython/gpython.h"
#include "models/least_squares/least_squares.h"
#include "models/field_propagation/field_propagation.h"
#include "models/baysian_inference/bayesian_inference.h"
#include "models/model_exporter.h"
#include "models/model.h"

ModelManager::ModelManager() {
    sys_cmd_sub_ = ros::Subscriber(
            Parameters::instance().node_handle_ptr_->subscribe("/syscommand", 10, &ModelManager::sysCmdCallback, this));
    reset_service_ = ros::ServiceServer(
            Parameters::instance().node_handle_ptr_->advertiseService("resetModel", &ModelManager::resetServiceCallback,
                                                                      this));

    ModelExporter::instance();

    Model& gp = GPython::instance();
    Model& ls = LeastSquares::instance();
    Model& fp = FieldPropagation::instance();
    Model& bi = BayesianInference::instance();

    models_ = {&gp, &ls, &fp, &bi};
    for (Model* model : models_) {
        model->activate();
    }
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
    for (Model* model : models_) {
        model->reset();
    }
}

void ModelManager::shutDown() {
    for (Model* model : models_) {
        model->shutDown();
    }
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

std::vector<Model *> ModelManager::getModels() const {
    return models_;
}
