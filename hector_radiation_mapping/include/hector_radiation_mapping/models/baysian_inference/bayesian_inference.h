#ifndef RADIATION_MAPPING_BAYESIAN_INFERENCE_H
#define RADIATION_MAPPING_BAYESIAN_INFERENCE_H

#include "hector_radiation_mapping/models/model.h"
#include "maps/gridmap.h"

/**
 * @brief The LeastSquares class
 */
class BayesianInference : public Model {
public:

    /**
     * Returns the instance of the LeastSquares class.
     * @return The instance of the LeastSquares class.
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

    void updateLoop() override;

    void paramCallback();

    /**
     * Set the minimum update time for the grid map, to limit data traffic.
     * @param time
     */
    void setMinUpdateTime(int time);

    /**
     * Callback for the slam map.
     * @param gridMsgPtr
     */
    void slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &gridMsgPtr);


    std::shared_ptr<GridMap> grid_map_;
    std::string layer_name_std_dev_;
    std::string layer_name_mean_;
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map_;
    volatile bool use_circle_;
    volatile int min_update_time_;
    std::shared_ptr<ros::Subscriber> slam_map_subscriber_;
};

#endif //RADIATION_MAPPING_BAYESIAN_INFERENCE_H
