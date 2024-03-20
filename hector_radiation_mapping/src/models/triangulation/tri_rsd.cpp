#include "models/triangulation/tri_rsd.h"
#include "util/util.h"
#include "util/parameters.h"
#include "util/clock.h"
#include "util/dddynamic_reconfigure.h"
#include "hector_radiation_mapping/sampleManager.h"

TriRSD::TriRSD() : Model(ModelType::TRI_RSD, Parameters::instance().tr_on_start_up, Parameters::instance().tr_min_update_time) {
    layerNameVal_ = "valueMap";
    layerNameErr_ = "errorMap";
    grid_map_ = std::make_shared<GridMap>(Parameters::instance().tr_grid_map_topic,
                                          Parameters::instance().tr_grid_map_resolution);
    grid_map_->addLayer(layerNameVal_);
    grid_map_->addLayer(layerNameErr_);

    useCircle_ = true;

    slam_map_subscriber_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe<nav_msgs::OccupancyGrid>(
                    Parameters::instance().environment_map_topic, 1, &TriRSD::slamMapCallback, this));

    /// set up dddynamicReconfigure
}

void TriRSD::reset() {

}

void TriRSD::update() {
    {
        std::unique_lock<std::mutex> lock{sample_queue_mtx_};
        update_condition_.wait(lock, [this]() { return (!samples_add_queue_.empty()) || !active_; });
        if (!active_) return;
    }
    updateSamples();

    // STUFF
    Clock clock;
    ROS_INFO_STREAM("TriRSD evaluate");
    evaluate();
    grid_map_->publish();
}

void TriRSD::evaluate() {
    Vector2d center = SampleManager::instance().getLastSamplePos().topRows(2);
    std::vector<Sample> samples = SampleManager::instance().getSamplesWithinRadius(center, 5.0);

    int n = samples.size();
    if (n < 3) {
        ROS_INFO_STREAM("TriRSD: Not enough samples to evaluate");
        return;
    }

    int m = 20;
    // select m strongest samples
    std::sort(samples.begin(), samples.end(), [](const Sample &a, const Sample &b) {
        return a.dose_rate_ > b.dose_rate_;
    });

    if (n > m) {
        n = m;
    }
    double avg_x = 0.0;
    double avg_y = 0.0;
    double avg_I = 0.0;
    int total = 0;
    // iterate over all triangles
    for(int i = 0; i < n - 2; i++){
        for(int ii = i + 1; ii < n - 1; ii++){
            for(int iii = ii + 1; iii < n; iii++){
                Sample &s1 = samples[i];
                Sample &s2 = samples[ii];
                Sample &s3 = samples[iii];
                double I1;
                double I2;
                double x1;
                double y1;
                double x2;
                double y2;
                if(Test::calculateA12(I1, I2, x1, y1, x2, y2, s1.dose_rate_, s2.dose_rate_, s3.dose_rate_,
                                      s1.position_.x(), s2.position_.x(), s3.position_.x(),
                                      s1.position_.y(), s2.position_.y(), s3.position_.y())){
                    //ROS_INFO_STREAM("TriRSD: A12" << I1 << " " << I2 << " " << x1 << " " << y1 << " " << x2 << " " << y2);
                    // check for nan
                    if(I1 != I1 || I2 != I2 || x1 != x1 || y1 != y1 || x2 != x2 || y2 != y2){
                        continue;
                    }
                    Vector2d p1(x1, y1);
                    Vector2d p2(x2, y2);
                    if((center - p1).squaredNorm() < (center - p2).squaredNorm()){
                        avg_I += I1;
                        avg_x += x1;
                        avg_y += y1;
                        total++;
                    } else {
                        avg_I += I2;
                        avg_x += x2;
                        avg_y += y2;
                        total++;
                    }
                }
            }
        }
    }

    if(total > 0){
        avg_x /= total;
        avg_y /= total;
        avg_I /= total;
    }

    for (TextMarker text_marker: text_markers_) {
        text_marker.deleteMarker();
    }
    Vector3d marker_pos(avg_x, avg_y, 0.0);
    ROS_INFO_STREAM("TriRSD: " << avg_I << " " << marker_pos.transpose());
    TextMarker text_marker(marker_pos, "TRI - " + std::to_string(avg_I), 0.2);
    text_markers_.push_back(text_marker);
}

void TriRSD::slamMapCallback(const nav_msgs::OccupancyGrid_<std::allocator<void>>::ConstPtr &grid_msg_ptr) {
    slam_map_ = std::make_shared<nav_msgs::OccupancyGrid>(*grid_msg_ptr);
}