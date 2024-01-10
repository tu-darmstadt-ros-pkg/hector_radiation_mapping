#include "hector_radiation_mapping/sampleManager.h"
#include "hector_radiation_mapping/source.h"
#include "hector_radiation_mapping/sample.h"
#include "util/parameters.h"
#include "models/model_manager.h"
#include "models/gpython/gpython.h"

SampleManager::SampleManager() {
    // Fix topic name if necessary
    if (Parameters::instance().subscribe_topic.at(0) != '/') {
        Parameters::instance().subscribe_topic = "/" + Parameters::instance().subscribe_topic;
    }

    // Init message parser
    babel_fish_ = std::make_shared<ros_babel_fish::BabelFish>();
    universal_sub_ = std::make_shared<ros::Subscriber>(
            Parameters::instance().node_handle_ptr_->subscribe<ros_babel_fish::BabelFishMessage>(
                    Parameters::instance().subscribe_topic,
                    Parameters::instance().dose_sub_size,
                    &SampleManager::doseCallbackFish, this
            ));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    // initialize vars
    background_radiation_cps_ = 0.0;
    background_radiation_dose_rate_ = 0.0;
    trajectory_marker_ = std::make_shared<TrajectoryMarker>();
}

SampleManager &SampleManager::instance() {
    static SampleManager instance;
    return instance;
}

void SampleManager::doseCallbackFish(const ros_babel_fish::BabelFishMessage::ConstPtr &msg) {
    std::string key_rate = Parameters::instance().message_key_rate;
    std::string key_cps = Parameters::instance().message_key_cps;
    std::string key_frame_id = Parameters::instance().message_key_frame_id;
    ros_babel_fish::TranslatedMessage::Ptr translated = babel_fish_->translateMessage(msg);
    auto &compound = translated->translated_message->as<ros_babel_fish::CompoundMessage>();
    double dose_rate = compound[key_rate].value<float>();
    double cps = -1;
    std::string frame_id = compound["header"][key_frame_id].value<std::string>();

    try {
        cps = compound[key_cps].value<float>();
    } catch (...) {
        cps = dose_rate * mikro_sievert_per_hour_to_cps_;
    }

    try {
        geometry_msgs::Transform trans = tf_buffer_.lookupTransform("world", frame_id, ros::Time(0),
                                                                    ros::Duration(1)).transform;
        Eigen::Vector3d pos(trans.translation.x, trans.translation.y, trans.translation.z);

        updateTrajectory(Parameters::instance().use_dose_rate ? dose_rate : cps, pos);

        static bool calculated_background_radiation = false;
        if (!calculated_background_radiation) {
            static std::vector<double> cps_vec;
            static std::vector<double> dose_rate_vec;
            static std::vector<Eigen::Vector3d> pos_vec;
            static std::vector<ros::Time> time_vec;
            cps_vec.push_back(cps);
            dose_rate_vec.push_back(dose_rate);
            pos_vec.push_back(pos);
            time_vec.push_back(ros::Time::now());
            background_radiation_cps_ += cps / 10.0;
            background_radiation_dose_rate_ += dose_rate / 10.0;

            if (cps_vec.size() >= 10) {
                for (int i = 0; i < cps_vec.size(); i++) {
                    processSampleData(pos_vec[i], cps_vec[i], dose_rate_vec[i], time_vec[i]);
                }
                cps_vec.clear();
                dose_rate_vec.clear();
                pos_vec.clear();
                time_vec.clear();
                calculated_background_radiation = true;
            }
        } else {
            processSampleData(pos, cps, dose_rate, ros::Time::now());
        }

        //double time = sample.time_.toSec() - Parameters::instance().startTime_;
        //std::string export_path = Util::getExportPath("samples");
        //Util::appendDoubleToTxtFile(cps, export_path, "cps");
        //Util::appendDoubleToTxtFile(dose_rate, export_path, "dose_rate");
        //Util::appendDoubleToTxtFile(time, export_path, "time");
    } catch (tf2::TransformException &exception) {
        ROS_WARN_STREAM(exception.what());
    }
}


void SampleManager::processSampleData(Vector3d pos, double cps, double dose_rate, ros::Time time) {
    /// Get background radiation from average of first few samples
    double adj_cps = fmax(cps - background_radiation_cps_, 0.0);
    double adj_dose_rate = fmax(dose_rate - background_radiation_dose_rate_, 0.0);
    Sample sample(pos, adj_cps, adj_dose_rate, time);

    if(!Parameters::instance().enable_spatial_sample_filtering){
        double min_dist2 = 0.2 * 0.2;
        if (samples_.empty() || (sample.position_ - getLastSamplePos()).squaredNorm() > min_dist2 || sample.doseRate_ > 20.0) {
            addSample(sample);
        }
        return;
    }

    const double dist_cutoff = 0.005;
    const double dist_cutoff2 = dist_cutoff * dist_cutoff;

    if (sample_queue_.empty()) {
        sample_queue_.push_back(sample);
    }
    else {
        /// if position has changed considerably, add new sample and the mean of the old ones that are left in the Queue
        if((sample.position_ - sample_queue_.back().position_).squaredNorm() > dist_cutoff2) {
            Sample mean_sample = getMeanSample(sample_queue_, false);
            for(int i = 0; i < ceil((double)sample_queue_.size() / 5.0); i++){
                addSample(mean_sample);
            }
            addSample(sample);
            sample_queue_.clear();
        }
        else {
            sample_queue_.push_back(sample);

            /// if position has not changed much, add mean of all Samples in the Queue if largest distance exceeds threshold
            double max_dist, dist_sqr = 0.0;
            for(int i = 0; i < sample_queue_.size() - 1; ++i) {
                for(int j = i + 1; j < sample_queue_.size(); j++){
                    dist_sqr = (sample_queue_[i].position_ - sample_queue_[j].position_).squaredNorm();
                    if(dist_sqr > max_dist){
                        max_dist = dist_sqr;
                    }
                }
            }
            if(max_dist > dist_cutoff2){
                Sample mean_sample = getMeanSample(sample_queue_, true);
                for(int i = 0; i < ceil((double)sample_queue_.size() / 5.0); i++){
                    addSample(mean_sample);
                }
                sample_queue_.clear();
            }
        }
    }
}

void SampleManager::addSample(Sample &sample) {
    samples_.push_back(sample);

    // Position with 2 decimals
    double pos_x = round(sample.position_[0] * 100) / 100;
    double pos_y = round(sample.position_[1] * 100) / 100;
    double pos_z = round(sample.position_[2] * 100) / 100;
    STREAM("Added sample with dose rate " << sample.doseRate_ << " " << Parameters::instance().radiation_unit
                            << " at [" << pos_x << ", " << pos_y << ", " << pos_z << "].\n");

    // Add sample to radiation models
    for (const auto &model: ModelManager::instance().getModels()) {
        model->addSample(sample);
    }
}

void SampleManager::updateTrajectory(const double &value, const Eigen::Vector3d &position) {
    trajectory_marker_->addPoint(value, position);
}

Eigen::Vector3d SampleManager::getLastSamplePos() {
    if (samples_.empty()) {
        return {-DBL_MAX, -DBL_MAX, -DBL_MAX};
    }
    return samples_.back().position_;
}

std::vector<Sample> SampleManager::getSamplesWithinRadius(const Eigen::Vector3d &position, double radius) {
    std::vector<Sample> in_sphere;
    double radius2 = radius * radius;
    for (const Sample &sample: samples_) {
        if ((position - sample.position_).squaredNorm() < radius2) {
            in_sphere.push_back(sample);
        }
    }
    return in_sphere;
}

Sample SampleManager::getNearestSample(const Eigen::Vector3d &position) {
    double min_dist = DBL_MAX;
    Sample nearest_sample;
    for (const Sample &sample: samples_) {
        double dist = (position - sample.position_).squaredNorm();
        if (dist < min_dist) {
            min_dist = dist;
            nearest_sample = sample;
        }
    }
    return nearest_sample;
}

Sample SampleManager::getMeanSample(const std::vector<Sample> &samples, bool weighted) {
    Eigen::Vector3d mean_pos(0.0, 0.0, 0.0);
    double mean_cps = 0.0;
    double mean_dose_rate = 0.0;
    double weight = 1.0;
    double total_weight = 0.0;

    for (const Sample &sample: samples) {
        mean_pos = mean_pos + weight * sample.position_;
        mean_cps = mean_cps + weight * sample.cps_;
        mean_dose_rate = mean_dose_rate + weight * sample.doseRate_;
        total_weight += weight;
        //weight += 0.5;
    }

    mean_pos /= total_weight;
    mean_cps /= total_weight;
    mean_dose_rate /= total_weight;

    Sample combined_sample(mean_pos, mean_cps, mean_dose_rate, ros::Time::now());
    return combined_sample;
}

std::vector<Sample> SampleManager::getSamplesNotInIdVector(const std::vector<Sample> &samples,
                                                           const std::vector<int> &ids) {
    std::vector<Sample> samples_not_in_id_vector;
    for (const Sample &sample: samples) {
        if (std::find(ids.begin(), ids.end(), sample.id_) == ids.end()) {
            samples_not_in_id_vector.push_back(sample);
        }
    }
    return samples_not_in_id_vector;
}

std::vector<Sample> SampleManager::getSamples() {
    return samples_;
}

tf2_ros::Buffer &SampleManager::getTfBuffer() {
    return tf_buffer_;
}

Sample SampleManager::getSampleById(int id) {
    for (const Sample &sample: samples_) {
        if (sample.id_ == id) {
            return sample;
        }
    }
    return Sample(); // return empty sample if no sample with id was found
}

void SampleManager::reset() {
    samples_.clear();
    sample_queue_.clear();
    background_radiation_cps_ = 0.0;
    background_radiation_dose_rate_ = 0.0;
    trajectory_marker_->reset();
}
