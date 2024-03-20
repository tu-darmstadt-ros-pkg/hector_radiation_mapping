#include "models/baysian_inference/bayesian_inference.h"
#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "hector_radiation_mapping/sampleManager.h"
#include <boost/math/distributions/poisson.hpp>

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


    // count probability
    for (int i = 0; i < positions.cols(); i++) {
        Vector2d position = positions.col(i);

        for(const Sample sample : this->samples_){
            double dist2 = (position - sample.get2DPos()).squaredNorm();
            double count_std_dev = sqrt(sample.dose_rate_);
            double dev_fac = 3.0;
            double c_upper = sample.dose_rate_ + dev_fac * count_std_dev;
            double c_lower = sample.dose_rate_ - dev_fac * count_std_dev;

            double time_fac = 1.0;
            double attenuation_fac = 1.0;
            double directionality_fac = 1.0;
            double mu = time_fac * attenuation_fac * directionality_fac;


            double p_upper = cumulativePoisson(c_upper, mu);
            double p_lower = cumulativePoisson(c_lower, mu);
            double p = p_upper - p_lower;
        }


    }
}

void BayesianInference::paramCallback() {

}

void BayesianInference::slamMapCallback(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &grid_msg_ptr) {
    slam_map_ = std::make_shared<nav_msgs::OccupancyGrid>(*grid_msg_ptr);
}

double BayesianInference::cumulativePoisson(double lambda, int k) {
    return boost::math::cdf(boost::math::poisson(lambda), k);
}