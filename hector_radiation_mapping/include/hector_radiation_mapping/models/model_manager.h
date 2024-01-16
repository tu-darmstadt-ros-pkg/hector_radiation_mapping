#ifndef RADIATION_MAPPING_MODEL_MANAGER_H
#define RADIATION_MAPPING_MODEL_MANAGER_H

#include "pch.h"

#include "models/model.h"

class ModelManager {
public:
    /**
     * Returns the class instance.
     * @return class instance.
     */
    static ModelManager& instance();

    /**
     * Resets the models. It deletes all markers and interactive markers.
     */
    void resetModels();

    /**
     * Shuts down the model manager. It shuts down the model exporter and the radiation models.
     */
    void shutDown();

    /**
     * Get all radiation models.
     * @return all radiation models
     */
    std::vector<Model*> getModels() const;

    void toggleModel(bool active, std::string model_name);

private:
    /**
     * Private constructor for singleton pattern.
     * It initializes the reset service and the syscommand subscriber. It also initializes the model exporter
     * and radiation models.
     */
    ModelManager();
    ModelManager(const ModelManager&) = delete;
    ModelManager& operator=(const ModelManager&) = delete;

    std::vector<Model*> models_;
};

#endif //RADIATION_MAPPING_MODEL_MANAGER_H