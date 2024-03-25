#ifndef RADIATION_MAPPING_LEAST_SQUARES_H
#define RADIATION_MAPPING_LEAST_SQUARES_H

#include "pch.h"

#include "hector_radiation_mapping/models/model.h"
#include "maps/gridmap.h"
#include "marker/marker.h"
/**
 * @brief The LeastSquares class
 */
class LeastSquares : public Model {
public:
    struct Result {
        std::vector<Vector2d> radius_minima;
        std::vector<Vector2d> latest_minima;
    };

    struct InverseSquareCostFunctor {
        InverseSquareCostFunctor(double x, double y, double intensity)
                : x_(x), y_(y), intensity_(intensity) {}

        template<typename T>
        bool operator()(const T* const x, const T* const y, const T* const intensity, T* residual) const {
            const T dx = x[0] - T(x_);
            const T dy = y[0] - T(y_);
            residual[0] = intensity[0] / (dx * dx + dy * dy) - intensity_;
            return true;
        }

    private:
        const double x_;
        const double y_;
        const double intensity_;
    };

    struct InverseAltSquareCostFunctor {
        explicit InverseAltSquareCostFunctor(std::vector<Sample> &samples)
                : samples_(samples) {}

        template<typename T>
        bool operator()(const T* const x, const T* const y, T* residual) const {
            T numerator = T(0);
            T denominator = T(0);
            for (const Sample &sample: samples_) {
                const T dx = x[0] - T(sample.get2DPos().x());
                const T dy = y[0] - T(sample.get2DPos().y());
                const T dist2_inv = (T(1) / (dx * dx + dy * dy));
                const T dist4_inv = dist2_inv * dist2_inv;
                numerator += T(sample.dose_rate_) * dist2_inv;
                denominator += dist4_inv;
            }
            const T intensity = numerator / denominator;
            T error = T(0);
            for (const Sample &sample: samples_) {
                const T dx = x[0] - T(sample.get2DPos().x());
                const T dy = y[0] - T(sample.get2DPos().y());
                const T dist2_inv = (T(1) / (dx * dx + dy * dy));
                const T error_part = (T(sample.dose_rate_) - intensity * dist2_inv);
                error += error_part * error_part;
            }
            residual[0] = error;
            return true;
        }

    private:
        std::vector<Sample> &samples_;
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

    std::shared_ptr<LeastSquares::Result> getResult();

private:
    LeastSquares();

    LeastSquares(const LeastSquares &) = delete;

    LeastSquares &operator=(const LeastSquares &) = delete;

    void update() override;

    void paramCallback();

    void evaluate3();

    void evaluate();

    /**
     * Callback for the slam map.
     * @param grid_msg_ptr
     */
    void slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg_ptr);

    void setMaxQueueSize(int max_queue_size);

    static void
    calculateOptimalIntensity(const Vector2d &position, const std::vector<Sample> &samples, float &intensity,
                              float &numerator, float &denominator);

    static void calculateError(const Vector2d &position, const std::vector<Sample> &samples, float intensity, float &error);

    void createMinMarkers(const Vector2d &center, const grid_map::GridMap &grid_map_ref, const grid_map::Matrix &layer_error, const Eigen::Vector4d &color);

    std::string layer_name_intensity_;
    std::string layer_name_error_;
    std::string layer_name_error_radius_;
    std::string layer_name_error_latest_;
    std::string layer_name_i_numerator_;
    std::string layer_name_i_denominator_;

    std::vector<TextMarker> text_markers_;
    ArrowMarker gradient_marker_;
    volatile std::atomic_bool use_circle_{};
    volatile std::atomic_uint current_queue_id_{};
    volatile std::atomic_uint max_queue_size_{};

    std::shared_ptr<GridMap> grid_map_;
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map_;
    std::shared_ptr<ros::Subscriber> slam_map_subscriber_;

    std::mutex result_mutex_;
    std::shared_ptr<Result> result_;
    std::shared_ptr<Result> new_result_;
};

#endif //RADIATION_MAPPING_LEAST_SQUARES_H
