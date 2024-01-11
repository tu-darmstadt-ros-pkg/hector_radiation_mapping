#ifndef RADIATION_MAPPING_FIELD_PROPAGATION_H
#define RADIATION_MAPPING_FIELD_PROPAGATION_H

#include "hector_radiation_mapping/models/model.h"
#include "maps/gridmap.h"

/**
 * @brief The LeastSquares class
 */
class FieldPropagation : public Model {
public:

    /**
     * Returns the class instance.
     * @return class instance.
     */
    static FieldPropagation &instance(){
        static FieldPropagation instance;
        return instance;
    }

    /**
     * Resets the model.
     */
    void reset() override;

    void evaluate(Matrix &positions);


private:
    FieldPropagation();
    FieldPropagation(const FieldPropagation &) = delete;
    FieldPropagation &operator=(const FieldPropagation &) = delete;

    void update() override;

    void paramCallback();

    /**
     * Callback for the slam map.
     * @param gridMsgPtr
     */
    void slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &gridMsgPtr);

    std::string layer_name_std_dev_;
    std::string layer_name_mean_;
    volatile std::atomic_bool use_circle_;

    std::shared_ptr<GridMap> grid_map_;
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map_;
    std::shared_ptr<ros::Subscriber> slam_map_subscriber_;
};

#endif //RADIATION_MAPPING_FIELD_PROPAGATION_H
