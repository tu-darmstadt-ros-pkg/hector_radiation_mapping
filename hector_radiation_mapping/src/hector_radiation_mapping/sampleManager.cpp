#include "hector_radiation_mapping/sampleManager.h"
#include "hector_radiation_mapping/source.h"
#include "hector_radiation_mapping/sample.h"
#include "util/parameters.h"
#include "models/model_manager.h"
#include "exploration/exploration_map.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

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
    ROS_INFO_STREAM("Received dose rate " << dose_rate << " " << Parameters::instance().radiation_unit << ".\n");
    double cps = -1;
    std::string frame_id = compound["header"][key_frame_id].value<std::string>();
    try {
        cps = compound[key_cps].value<float>();
    } catch (...) {
        cps = dose_rate * mikro_sievert_per_hour_to_cps_;
    }

    try {
        geometry_msgs::Transform sensor_trans = tf_buffer_.lookupTransform("world", frame_id, ros::Time(0),
                                                                           ros::Duration(1)).transform;
        geometry_msgs::Transform base_trans = tf_buffer_.lookupTransform("world", Parameters::instance().robot_base_frame, ros::Time(0),
                                                                    ros::Duration(1)).transform;

        Eigen::Vector3d sensor_pos(sensor_trans.translation.x, sensor_trans.translation.y, sensor_trans.translation.z);
        Eigen::Vector3d base_pos(base_trans.translation.x, base_trans.translation.y, base_trans.translation.z);

        updateTrajectory(Parameters::instance().use_dose_rate ? dose_rate : cps, sensor_pos);

        static bool calculated_background_radiation = Parameters::instance().background_radiation_dose_rate_set;
        if (!calculated_background_radiation) {
            static std::vector<double> cps_vec;
            static std::vector<double> dose_rate_vec;
            static std::vector<Vector3d> sensor_pos_vec;
            static std::vector<Vector3d> base_pos_vec;
            static std::vector<ros::Time> time_vec;
            cps_vec.push_back(cps);
            dose_rate_vec.push_back(dose_rate);
            sensor_pos_vec.push_back(sensor_pos);
            base_pos_vec.push_back(base_pos);
            time_vec.push_back(ros::Time::now());
            background_radiation_cps_ += cps / 10.0;
            background_radiation_dose_rate_ += dose_rate / 10.0;

            if (cps_vec.size() >= 10) {
                for (int i = 0; i < cps_vec.size(); i++) {
                    processSampleData(sensor_pos_vec[i], base_pos_vec[i], cps_vec[i], dose_rate_vec[i], time_vec[i]);
                }
                cps_vec.clear();
                dose_rate_vec.clear();
                sensor_pos_vec.clear();
                base_pos_vec.clear();
                time_vec.clear();
                calculated_background_radiation = true;
            }
        } else {
            ROS_INFO_STREAM("Received dose rate2 " << dose_rate << " " << Parameters::instance().radiation_unit << ".\n");
            processSampleData(sensor_pos, base_pos, cps, dose_rate, ros::Time::now());
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

void SampleManager::processSampleData(Vector3d sensor_position, Vector3d base_position, double cps, double dose_rate, ros::Time time) {
    /// Get background radiation from average of first few samples
    double adj_cps = fmax(cps - background_radiation_cps_, 0.0);
    double adj_dose_rate = fmax(dose_rate - background_radiation_dose_rate_, 0.0);
    Sample sample(sensor_position, base_position, adj_cps, adj_dose_rate, time);
    ROS_INFO_STREAM("Received sample with dose rate " << sample.dose_rate_ << " at [" << sensor_position.x() << ", " << sensor_position.y() << ", " << sensor_position.z() << "].\n");

    if(Parameters::instance().enable_spatial_sample_filtering){
        double min_dist2 = 0.2 * 0.2;
        /*
        if (samples_.empty() || (sample.sensor_position_ - getLastSamplePos()).squaredNorm() > min_dist2 || sample.dose_rate_ > 20.0) {
            addSample(sample);
        }*/
        addSample(sample);
        return;
    }

    const double dist_cutoff = 0.005;
    const double dist_cutoff2 = dist_cutoff * dist_cutoff;

    if (sample_queue_.empty()) {
        sample_queue_.push_back(sample);
    }
    else {
        /// if position has changed considerably, add new sample and the mean of the old ones that are left in the Queue
        if((sample.sensor_position_ - sample_queue_.back().sensor_position_).squaredNorm() > dist_cutoff2) {
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
                    dist_sqr = (sample_queue_[i].sensor_position_ - sample_queue_[j].sensor_position_).squaredNorm();
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
    ExplorationMap::instance().addSampleLocation(sample.get2DPos());

    samples_.push_back(sample);
    rtree_.insert(sample);

    // Position with 2 decimals
    double pos_x = round(sample.sensor_position_[0] * 100) / 100;
    double pos_y = round(sample.sensor_position_[1] * 100) / 100;
    double pos_z = round(sample.sensor_position_[2] * 100) / 100;
    ROS_INFO_STREAM("Added sample with dose rate " << sample.dose_rate_ << " " << Parameters::instance().radiation_unit
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
    return samples_.back().sensor_position_;
}

std::vector<Sample> SampleManager::getSamplesWithinRadius(const Eigen::Vector3d &position, double radius2) {
    std::vector<Sample> result_rtree;
    rtree_.query(bgi::satisfies([&position, radius2](const Sample &s) {
        return (position - s.sensor_position_).squaredNorm() < radius2;
    }), std::back_inserter(result_rtree));
    return result_rtree;
}

std::vector<Sample> SampleManager::getSamplesWithinRadius(const Vector2d &position, double radius2) {
    std::vector<Sample> result_rtree;
    rtree_.query(bgi::satisfies([&position, radius2](const Sample &s) {
        return (position - s.get2DPos()).squaredNorm() < radius2;
    }), std::back_inserter(result_rtree));
    return result_rtree;
}

Sample SampleManager::getNearestSample(const Eigen::Vector3d &position) {
    auto min_dist = DBL_MAX;
    Sample nearest_sample;
    for (const Sample &sample: samples_) {
        double dist = (position - sample.sensor_position_).squaredNorm();
        if (dist < min_dist) {
            min_dist = dist;
            nearest_sample = sample;
        }
    }
    return nearest_sample;
}

Sample SampleManager::getMeanSample(const std::vector<Sample> &samples, bool weighted) {
    Vector3d mean_sensor_pos(0.0, 0.0, 0.0);
    Vector3d mean_base_pos(0.0, 0.0, 0.0);
    double mean_cps = 0.0;
    double mean_dose_rate = 0.0;
    double weight = 1.0;
    double total_weight = 0.0;

    for (const Sample &sample: samples) {
        mean_sensor_pos = mean_sensor_pos + weight * sample.sensor_position_;
        mean_base_pos = mean_base_pos + weight * sample.base_position_;
        mean_cps = mean_cps + weight * sample.cps_;
        mean_dose_rate = mean_dose_rate + weight * sample.dose_rate_;
        total_weight += weight;
        //weight += 0.5;
    }

    mean_sensor_pos /= total_weight;
    mean_base_pos /= total_weight;
    mean_cps /= total_weight;
    mean_dose_rate /= total_weight;

    Sample combined_sample(mean_sensor_pos, mean_base_pos, mean_cps, mean_dose_rate, ros::Time::now());
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
    return {}; // return empty sample if no sample with id was found
}

void SampleManager::reset() {
    samples_.clear();
    sample_queue_.clear();
    background_radiation_cps_ = 0.0;
    background_radiation_dose_rate_ = 0.0;
    trajectory_marker_->reset();
}

std::vector<Sample> SampleManager::getLatestSamples(int num_samples) {
    if (samples_.size() < num_samples) {
        return samples_;
    }
    return std::vector<Sample>(samples_.end() - num_samples, samples_.end());
}

Vector2d SampleManager::getRadiationGradient(const Vector2d &position, double radius) {
    std::vector<Sample> samples = SampleManager::instance().getSamplesWithinRadius(position, radius * radius);
    if (samples.size() <= 2) {
        return {0, 0};
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
        Vector2d grad = res.topRows(2);
        // normalize gradient
        grad.normalize();
        return grad;
    }
    return {0, 0};
}

double SampleManager::getMinDistanceToAllSamples(const Vector2d& position) {
    double min_dist = DBL_MAX;
    for (const Sample &sample: samples_) {
        double dist = (position - sample.get2DPos()).norm();
        if (dist < min_dist) {
            min_dist = dist;
        }
    }
    return min_dist;
}
