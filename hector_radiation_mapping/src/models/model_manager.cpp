#include "marker/marker_manager.h"
#include "util/dddynamic_reconfigure.h"
#include "models/model_manager.h"
#include "models/gpython/gpython.h"
#include "models/gpython/gpython2D.h"
#include "models/gpython/gpython3D.h"
#include "models/least_squares/least_squares.h"
#include "models/field_propagation/field_propagation.h"
#include "models/baysian_inference/bayesian_inference.h"
#include "models/model_exporter.h"
#include "models/model.h"

ModelManager::ModelManager() {
    std::string group_name_ = "MODEL_MANAGER";
    ModelExporter::instance();
    Model &gp = GPython::instance();
    GPython2D::instance();
    GPython3D::instance();
    Model &ls = LeastSquares::instance();
    Model &fp = FieldPropagation::instance();
    Model &bi = BayesianInference::instance();

    models_ = {&gp, &ls, &fp, &bi};
    for (Model *model: models_) {
        model->activate();
        std::string model_name = model->getModelName();
        DDDynamicReconfigure::instance().registerVariable<bool>(model_name, true,
                                                                boost::bind(&ModelManager::toggleModel, this, _1, model_name), "on/off", false,
                                                                true, group_name_);
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