#include "marker/marker_manager.h"
#include "util/dddynamic_reconfigure.h"
#include "models/model_manager.h"
#include "models/gpython/gaussian_process.h"
#include "models/gpython/gaussian_process_2d.h"
#include "models/gpython/gaussian_process_3d.h"
#include "models/least_squares/least_squares.h"
#include "models/field_propagation/field_propagation.h"
#include "models/baysian_inference/bayesian_inference.h"
#include "models/model_exporter.h"
#include "models/model.h"

ModelManager::ModelManager() {
    std::string group_name_ = "MODEL_MANAGER";
    ModelExporter::instance();
    Model &gp = GaussianProcess::instance();
    GaussianProcess2D::instance();
    GaussianProcess3D::instance();
    Model &ls = LeastSquares::instance();
    Model &fp = FieldPropagation::instance();
    Model &bi = BayesianInference::instance();

    models_ = {&ls, &fp, &gp, &bi};
    for (Model *model: models_) {
        bool on_start_up = model->onStartup();
        std::string model_name = model->getModelName();
        DDDynamicReconfigure::instance().registerVariable<bool>(model_name, on_start_up,
                                                                boost::bind(&ModelManager::toggleModel, this, _1, model_name), "on/off", false,
                                                                true, group_name_);
        if (on_start_up) {
            model->activate();
        }
    }
    DDDynamicReconfigure::instance().publish();
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
    for (Model *model: models_) {
        model->reset();
    }
}

void ModelManager::shutDown() {
    for (Model *model: models_) {
        model->shutDown();
    }
}

std::vector<Model *> ModelManager::getModels() const {
    return models_;
}

void ModelManager::toggleModel(bool active, std::string model_name) {
    for (Model *model: models_) {
        if (model->getModelName() == model_name) {
            if (active) {
                model->activate();
            } else {
                model->deactivate();
            }
        }
    }
}