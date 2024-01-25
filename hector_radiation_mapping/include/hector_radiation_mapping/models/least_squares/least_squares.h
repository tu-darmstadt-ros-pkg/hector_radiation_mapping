#ifndef RADIATION_MAPPING_LEAST_SQUARES_H
#define RADIATION_MAPPING_LEAST_SQUARES_H

#include "pch.h"

#include "hector_radiation_mapping/models/model.h"
#include "maps/gridmap.h"

/**
 * @brief The LeastSquares class
 */
class LeastSquares : public Model {
public:

    /**
     * Struct for locally storing a sample and the information
     */
    struct SampleLS {
        SampleLS(Sample sample, u_int queue_id) : sample(std::move(sample)), queue_id_(queue_id) {};
        Sample sample;
        u_int queue_id_;
        bool active_ = false;
    };


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

    void samplesToSamplesLS(std::vector<Sample> &samples);

    void evaluate2();

    void evaluate3();

    /**
     * Callback for the slam map.
     * @param gridMsgPtr
     */
    void slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &gridMsgPtr);

    void setMaxQueueSize(int max_queue_size);

    static void
    calculateOptimalIntensity(const Vector2d &position, const std::vector<Sample> &samples, float &intensity,
                              float &numerator, float &denominator, bool add = true);

    static void
    calculateOptimalIntensity(const Vector2d &position, const std::vector<SampleLS> &samples, float &intensity,
                              float &numerator, float &denominator, bool add = true);

    void
    calculateOptimalIntensity(const Vector2d &position, const std::vector<SampleLS> &samples_add,
                              const std::vector<SampleLS> &samples_remove, float &intensity,
                              float &numerator, float &denominator, float &nnumerator, float &ndenominator);

    std::string layer_name_intensity_;
    std::string layer_name_error_;
    std::string layer_name_i_numerator_;
    std::string layer_name_i_denominator_;
    std::string layer_name_i_nnumerator_;
    std::string layer_name_i_ndenominator_;
    volatile std::atomic_bool use_circle_{};
    volatile std::atomic_uint current_queue_id_{};
    volatile std::atomic_uint max_queue_{};

    double max_dist;
    double min_dist;
    double max_num;
    double min_num;
    double max_den;
    double min_den;
    double max_int;
    double min_int;
    double min_err;
    double max_err;

    std::vector<SampleLS> samples_ls_;
    std::shared_ptr<GridMap> grid_map_;
    std::shared_ptr<GridMap> grid_map2_;
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map_;
    std::shared_ptr<ros::Subscriber> slam_map_subscriber_;
};


#endif //RADIATION_MAPPING_LEAST_SQUARES_H
