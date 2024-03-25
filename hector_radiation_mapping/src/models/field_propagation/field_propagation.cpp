#include <numeric>
#include "models/field_propagation/field_propagation.h"
#include "util/parameters.h"
#include "util/print.h"
#include "util/dddynamic_reconfigure.h"
#include "hector_radiation_mapping/sampleManager.h"

FieldPropagation::FieldPropagation() : Model(ModelType::FIELD_PROPAGATION, Parameters::instance().fp_on_start_up, Parameters::instance().fp_min_update_time) {
    layer_name_mean_ = "mean";
    layer_name_mean_confidence_ = "mean_confidence";
    layer_name_prop_oder_ = "prop_order";
    layer_name_prop_mean_ = "prop_mean";
    layer_name_prop_mean_confidence_ = "prop_mean_confidence";
    grid_map_ = std::make_shared<GridMap>(Parameters::instance().fp_grid_map_topic,
                                          Parameters::instance().fp_grid_map_resolution);
    grid_map_->addLayer(layer_name_mean_);
    grid_map_->addLayer(layer_name_mean_confidence_);
    grid_map_->addLayer(layer_name_prop_oder_);
    grid_map_->addLayer(layer_name_prop_mean_);
    grid_map_->addLayer(layer_name_prop_mean_confidence_);
    use_circle_ = false;

    slam_map_subscriber_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe<nav_msgs::OccupancyGrid>(
                    Parameters::instance().environment_map_topic, 1, &FieldPropagation::slamMapCallback, this));
}

void FieldPropagation::reset() {

}

void FieldPropagation::update() {
    updateSamples();
    static Matrix samplePositions;
    static bool use_circle = use_circle_;
    static Vector2d position = SampleManager::instance().getLastSamplePos().topRows(2);
    {
        std::lock_guard<std::mutex> lock{grid_map_->getGridMapMutex()};
        samplePositions = grid_map_->getSamplePositions(slam_map_, use_circle, Parameters::instance().fp_circle_radius,
                                                        position);
    }
    evaluate(samplePositions);
}

void FieldPropagation::evaluate(Matrix &positions) {
    Clock clock;
    std::lock_guard<std::mutex> lock{grid_map_->getGridMapMutex()};
    grid_map::Matrix &layer_mean = (grid_map_->getGridMapRef())[layer_name_mean_];
    grid_map::Matrix &layer_mean_confidence = (grid_map_->getGridMapRef())[layer_name_mean_confidence_];

    Eigen::Matrix<Vector2d, Eigen::Dynamic, Eigen::Dynamic> gradients(layer_mean.rows(), layer_mean.cols());
    Eigen::Matrix<Vector2d, Eigen::Dynamic, Eigen::Dynamic> entropy(layer_mean.rows(), layer_mean.cols());
    std::vector<Eigen::Vector2i> current_layer;

    // 4.1 Field Estimation
    // Measured Gradient Estimation
    for (int col = 0; col < layer_mean.cols(); ++col) {
        for (int row = 0; row < layer_mean.rows(); ++row) {
            int index = col * layer_mean.rows() + row;
            Vector2d position = positions.row(index);

            std::vector<Sample> samples = SampleManager::instance().getSamplesWithinRadius(position, Parameters::instance().fp_grid_map_resolution * 8);
            if (samples.size() <= 4) {
                layer_mean(row, col) = 0;
                layer_mean_confidence(row, col) = 0;
                gradients(row, col) = Vector2d(0, 0);
                continue;
            }

            // Construct X_ijk and S_ijk
            Eigen::MatrixX3d X_ijk;// N rows x 3 cols
            X_ijk.resize(samples.size(), Eigen::NoChange);
            Vector S_ijk(samples.size()); // N rows x 1 col
            Vector2d avg_pos(0, 0);
            for (int i = 0; i < samples.size(); ++i) {
                // extend sample pos 2d with 1 for 3D
                avg_pos += samples[i].sensor_position_.topRows(2);
                X_ijk.row(i) = Vector3d(samples[i].sensor_position_.x(), samples[i].sensor_position_.y(), 1).transpose();
                S_ijk(i) = samples[i].dose_rate_;
            }
            avg_pos /= samples.size();
            // subtract avg_pos from each row of X_ijk
            for (int i = 0; i < samples.size(); ++i) {
                X_ijk.row(i) -= Vector3d(avg_pos.x(), avg_pos.y(), 0).transpose();
            }

            // Check if determinant of X_ijk_transpose * X_ijk is zero (3x3)
            Eigen::Matrix3d XtX = X_ijk.transpose() * X_ijk;
            if (XtX.determinant() != 0) {
                Vector3d res = XtX.inverse() * X_ijk.transpose() * S_ijk;
                gradients(row, col) = res.topRows(2);
                layer_mean(row, col) = res(2);

                // TODO calculate Hijk Shannon Entropy
            } else {
                // get average of S_ijk
                layer_mean(row, col) = S_ijk.mean();
                gradients(row, col) = Vector2d(0, 0);
            }

            layer_mean_confidence(row, col) = 1.0 - 3.0 / samples.size();

            if (layer_mean(row, col) < SampleManager::instance().getBackgroundRadiationDoseRate()) {
                gradients(row, col) = Vector2d(0, 0);
                layer_mean(row, col) = 0;
            }

            current_layer.emplace_back(row, col);
        }
    }

    // 4.2 Field Propagation
    // Cell Mean Level Derivation
    Eigen::MatrixXf layer_prop_order = Eigen::MatrixXf::Constant(layer_mean.rows(), layer_mean.cols(), std::numeric_limits<float>::quiet_NaN());
    std::vector<std::vector<Eigen::Vector2i>> propagation_ordering;
    std::vector<Eigen::Vector2i> next_layer;
    int order = 0;

    while (!current_layer.empty()) {
        propagation_ordering.emplace_back();
        for (const Eigen::Vector2i &cell: current_layer) {
            int row = cell.x();
            int col = cell.y();

            if (!std::isnan(layer_prop_order(row, col))){
                continue;
            }
            layer_prop_order(row, col) = (float) order;
            propagation_ordering[order].emplace_back(row, col);

            // get neighbours
            if (row > 0) {
                next_layer.emplace_back(row - 1, col);
            }
            if (row < layer_mean.rows() - 1) {
                next_layer.emplace_back(row + 1, col);
            }
            if (col > 0) {
                next_layer.emplace_back(row, col - 1);
            }
            if (col < layer_mean.cols() - 1) {
                next_layer.emplace_back(row, col + 1);
            }
        }

        current_layer = next_layer;
        next_layer.clear();
        order++;
    }

    (grid_map_->getGridMapRef())[layer_name_prop_oder_] = layer_prop_order;

    // Mean Propagation given derived mean propagation ordering
    order = 0;
    grid_map::Matrix &layer_prop_mean = (grid_map_->getGridMapRef())[layer_name_prop_mean_];
    grid_map::Matrix &layer_prop_mean_confidence = (grid_map_->getGridMapRef())[layer_name_prop_mean_confidence_];
    Eigen::Matrix<Vector2d, Eigen::Dynamic, Eigen::Dynamic> prop_gradients(layer_mean.rows(), layer_mean.cols());
    float resolution = Parameters::instance().fp_grid_map_resolution;
    std::vector<Eigen::Vector2i> neighbours;

    for(std::vector<Eigen::Vector2i> &layer : propagation_ordering){
        for (const Eigen::Vector2i &cell: layer) {
            int row = cell.x();
            int col = cell.y();

            if (order == 0){
                layer_prop_mean(row, col) = layer_mean(row, col);
                layer_prop_mean_confidence(row, col) = layer_mean_confidence(row, col);
                prop_gradients(row, col) = gradients(row, col);
            } else {
                neighbours.clear();

                // get neighbours
                if (row > 0) {
                    neighbours.emplace_back(row - 1, col);
                }
                if (row < layer_mean.rows() - 1) {
                    neighbours.emplace_back(row + 1, col);
                }
                if (col > 0) {
                    neighbours.emplace_back(row, col - 1);
                }
                if (col < layer_mean.cols() - 1) {
                    neighbours.emplace_back(row, col + 1);
                }

                float contribution = 0.0;
                float weight = 0.0;
                float sum_weights = 0.0;
                Eigen::Vector2d gradient(0, 0);
                for (const Eigen::Vector2i &neighbour: neighbours) {
                    int n_row = neighbour.x();
                    int n_col = neighbour.y();

                    if(layer_prop_order(n_row, n_col) < (float)order){
                        weight = layer_prop_mean_confidence(n_row, n_col);
                        contribution += weight * (layer_prop_mean(n_row, n_col) +
                                (row - n_row) * prop_gradients(n_row, n_col).x() * resolution +
                                (col - n_col) * prop_gradients(n_row, n_col).y() * resolution);
                        sum_weights += weight;
                        gradient += weight * prop_gradients(n_row, n_col);
                    }
                }
                layer_prop_mean_confidence(row, col) = sum_weights / (float)neighbours.size();
                layer_prop_mean(row, col) = std::max(0.0f, contribution / sum_weights);
                prop_gradients(row, col) = 0.8 * gradient / sum_weights;
            }
        }
        order++;
    }


    grid_map_->publish();

    long time = clock.tock();
    ROS_INFO_STREAM("Field Propagation evaluation took " << time << " ms");
}

void FieldPropagation::paramCallback() {

}

void
FieldPropagation::slamMapCallback(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &grid_msg_ptr) {
    slam_map_ = std::make_shared<nav_msgs::OccupancyGrid>(*grid_msg_ptr);
}