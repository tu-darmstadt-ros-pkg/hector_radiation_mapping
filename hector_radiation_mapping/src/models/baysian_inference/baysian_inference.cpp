#include "models/baysian_inference/bayesian_inference.h"
#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "hector_radiation_mapping/sampleManager.h"

BayesianInference::BayesianInference() : Model(ModelType::BAYESIAN_INFERENCE, Parameters::instance().bi_on_start_up, Parameters::instance().bi_min_update_time) {
    layer_name_mean_ = "mean";
    grid_map_ = std::make_shared<GridMap>(Parameters::instance().bi_grid_map_topic,
                                          Parameters::instance().bi_grid_map_resolution);
    grid_map_->addLayer(layer_name_mean_);
    use_circle_ = true;

    slam_map_subscriber_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe<nav_msgs::OccupancyGrid>(
                    Parameters::instance().environment_map_topic, 1, &BayesianInference::slamMapCallback, this));
}

void BayesianInference::reset() {

}

void BayesianInference::update() {
    Matrix samplePositions;
    bool use_circle = use_circle_;
    Vector2d center = SampleManager::instance().getLastSamplePos().topRows(2);
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map = slam_map_;
    {
        std::lock_guard<std::mutex> lock{grid_map_->getGridMapMutex()};
        samplePositions = grid_map_->getSamplePositions(slam_map, use_circle, Parameters::instance().ls_circle_radius, center);
    }
    evaluate(samplePositions);
}

void BayesianInference::evaluate(Matrix &positions) {

}

void BayesianInference::paramCallback() {

}

void BayesianInference::slamMapCallback(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &grid_msg_ptr) {
    slam_map_ = std::make_shared<nav_msgs::OccupancyGrid>(*grid_msg_ptr);
}

