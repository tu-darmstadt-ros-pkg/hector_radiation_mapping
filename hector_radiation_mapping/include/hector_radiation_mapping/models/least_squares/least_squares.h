#ifndef RADIATION_MAPPING_LEAST_SQUARES_H
#define RADIATION_MAPPING_LEAST_SQUARES_H

#include "hector_radiation_mapping/models/model.h"
#include "maps/gridmap.h"

/**
 * @brief The LeastSquares class
 */
class LeastSquares : public Model {
public:

    struct CostFunctor {
        template<typename T>
        bool operator()(const T *x, T *residual) const;
    };

    /**
     * Returns the class instance.
     * @return class instance.
     */
    static LeastSquares &instance() {
        static LeastSquares instance;
        return instance;
    }

    /**
     * Resets the model.
     */
    void reset() override;

    void evaluate(Matrix &positions);


private:
    LeastSquares();

    LeastSquares(const LeastSquares &) = delete;

    LeastSquares &operator=(const LeastSquares &) = delete;

    void update() override;

    void paramCallback();

    /**
     * Callback for the slam map.
     * @param gridMsgPtr
     */
    void slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &gridMsgPtr);

    static void
    calculateOptimalIntensity(const Vector2d &position, const std::vector<Sample> &samples, float &intensity,
                              float &numerator, float &denominator);

    std::string layer_name_intensity_;
    std::string layer_name_error_;
    std::string layer_name_i_numerator_;
    std::string layer_name_i_denominator_;
    volatile std::atomic_bool use_circle_;

    std::shared_ptr<GridMap> grid_map_;
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map_;
    std::shared_ptr<ros::Subscriber> slam_map_subscriber_;
};


#endif //RADIATION_MAPPING_LEAST_SQUARES_H
