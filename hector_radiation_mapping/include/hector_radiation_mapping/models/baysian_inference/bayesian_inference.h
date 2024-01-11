#ifndef RADIATION_MAPPING_BAYESIAN_INFERENCE_H
#define RADIATION_MAPPING_BAYESIAN_INFERENCE_H

#include "hector_radiation_mapping/models/model.h"
#include "maps/gridmap.h"

/**
 * @brief The BayesianInference class
 */
class BayesianInference : public Model {
public:

    /**
     * Returns the class instance.
     * @return class instance.
     */
    static BayesianInference &instance(){
        static BayesianInference instance;
        return instance;
    }

    /**
     * Resets the model.
     */
    void reset() override;

    void evaluate(Matrix &positions);


private:
    BayesianInference();
    BayesianInference(const BayesianInference &) = delete;
    BayesianInference &operator=(const BayesianInference &) = delete;

    void update() override;

    void paramCallback();

    /**
     * Callback for the slam map.
     * @param gridMsgPtr
     */
    void slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &gridMsgPtr);

    std::string layer_name_std_dev_;
    std::string layer_name_mean_;
    volatile std::atomic_bool use_circle_{};

    std::shared_ptr<GridMap> grid_map_;
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map_;
    std::shared_ptr<ros::Subscriber> slam_map_subscriber_;
};

#endif //RADIATION_MAPPING_BAYESIAN_INFERENCE_H
