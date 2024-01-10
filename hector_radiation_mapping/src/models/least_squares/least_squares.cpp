#include "models/least_squares/least_squares.h"
#include "util/parameters.h"
#include "util/dddynamic_reconfigure.h"
#include "hector_radiation_mapping/sampleManager.h"

LeastSquares::LeastSquares() : Model(ModelType::LEAST_SQUARES) {
    layer_name_mean_ = "mean";
    grid_map_ = std::make_shared<GridMap>(Parameters::instance().ls_grid_map_topic,
                                          Parameters::instance().ls_grid_map_resolution);
    grid_map_->addLayer(layer_name_mean_);
    use_circle_ = true;
    min_update_time_ = Parameters::instance().ls_min_update_time;

    slam_map_subscriber_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe<nav_msgs::OccupancyGrid>(
                    Parameters::instance().environment_map_topic, 1, &LeastSquares::slamMapCallback, this));

    DDDynamicReconfigure::instance().registerVariable<int>(getShortModelName() + "_minUpdateTime",
                                                           min_update_time_,
                                                           boost::bind(&LeastSquares::setMinUpdateTime, this, _1),
                                                           "min/max",
                                                           0, 5000, getShortModelName());
    DDDynamicReconfigure::instance().publish();
}

void LeastSquares::reset() {

}

void LeastSquares::updateLoop() {
    Clock clock;
    while (active_ && ros::ok()) {
        clock.tick();
        bool use_circle = use_circle_;
        Vector2d center = SampleManager::instance().getLastSamplePos().topRows(2);
        std::shared_ptr<nav_msgs::OccupancyGrid> slam_map = slam_map_;
        {
            std::lock_guard<std::mutex> lock{grid_map_->getGridMapMutex()};
            //Matrix samplePositions = getSamplePositions(slam_map_, use_circle_, center);
        }

        int sleepTime = std::max(0, min_update_time_ - (int) clock.tock());
        std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
    }
}

void LeastSquares::evaluate(Matrix &positions) {

}

void LeastSquares::paramCallback() {

}

void LeastSquares::setMinUpdateTime(int time) {
    STREAM(modelTypeToName(model_type_) + " set min map update time = " << time);
    min_update_time_ = time;
}

void LeastSquares::slamMapCallback(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &grid_msg_ptr) {
    slam_map_ = std::make_shared<nav_msgs::OccupancyGrid>(*grid_msg_ptr);
}
