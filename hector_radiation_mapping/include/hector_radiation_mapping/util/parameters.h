#ifndef RADIATION_MAPPING_PARAMETERS_H
#define RADIATION_MAPPING_PARAMETERS_H

#include "pch.h"

class Parameters {
public:
    static Parameters &instance() {
        static Parameters instance;
        return instance;
    }

    std::shared_ptr<ros::NodeHandle> node_handle_ptr_;
    double startTime_;

    // General
    int dose_sub_size;
    int ros_spinner_threads;
    bool enable_online_3d_evaluation;
    std::string export_path;
    std::string world_frame;
    bool enable_spatial_sample_filtering;
    double background_radiation_dose_rate;
    double background_radiation_cps;
    bool background_radiation_dose_rate_set;
    bool background_radiation_cps_set;

    // ROS topics
    bool use_dose_rate;
    std::string subscribe_topic;
    std::string environment_cloud_topic;
    std::string environment_map_topic;
    std::string message_key_rate;
    std::string message_key_cps;
    std::string message_key_frame_id;
    std::string radiation_unit;

    // Gaussian Process
    // Source prediction
    bool gp_on_start_up;
    double gp_mean_factor;
    double gp_min_source_strength;
    // 2D model
    double gp_circle_radius;
    double gp_min_distance_between_samples_2d;
    int gp_min_update_time_2d;
    // GridMap
    std::string gp_grid_map_topic;
    double gp_grid_map_resolution;
    // 3D model
    double gp_local_radius;
    double gp_min_distance_between_samples_3d;
    int gp_min_update_time_3d;
    // 3D Point cloud
    std::vector<double> gp_distance_cutoff_levels;
    std::vector<int> gp_point_cloud_3d_size_levels;

    // Least Squares
    bool ls_on_start_up;
    double ls_circle_radius;
    int ls_min_update_time;
    std::string ls_grid_map_topic;
    double ls_grid_map_resolution;
    int ls_max_queue;

    // Bayesian Inference
    bool bi_on_start_up;
    double bi_circle_radius;
    int bi_min_update_time;
    std::string bi_grid_map_topic;
    double bi_grid_map_resolution;

    // Field Propagation
    bool fp_on_start_up;
    double fp_circle_radius;
    int fp_min_update_time;
    std::string fp_grid_map_topic;
    double fp_grid_map_resolution;

private:

    // templated method for loading a parameter from the parameter server
    template<typename T>
    bool loadParam(const std::string &paramName, T &param, const T &defaultValue = T()) {
        std::string completeName = "/hector_radiation_mapping/" + paramName;
        if (!node_handle_ptr_->param(completeName, param, defaultValue)) {
            ROS_ERROR("Could not load parameter %s using default value", completeName.c_str());
            return false;
        }
        return true;
    }

    /**
     * Constructor for the Parameters class.
     * Loads all parameters from the parameter server.
     */
    Parameters() {
        node_handle_ptr_ = std::make_shared<ros::NodeHandle>("hector_radiation_mapping");

        startTime_ = ros::Time::now().toSec();
        dose_sub_size = 100;

        // General
        loadParam("ros_spinner_threads", ros_spinner_threads);
        loadParam("enable_online_3d_evaluation", enable_online_3d_evaluation);
        loadParam("enable_spatial_sample_filtering", enable_spatial_sample_filtering);
        loadParam("export_path", export_path);
        loadParam("world_frame", world_frame);
        background_radiation_dose_rate_set = loadParam("background_radiation_dose_rate", background_radiation_dose_rate);
        background_radiation_cps_set = loadParam("background_radiation_cps", background_radiation_cps);

        // ROS topics
        loadParam("subscribe_topic", subscribe_topic);
        loadParam("environment_map_topic", environment_map_topic);
        loadParam("environment_cloud_topic", environment_cloud_topic);
        loadParam("message_key_rate", message_key_rate);
        loadParam("message_key_cps", message_key_cps);
        loadParam("message_key_frame_id", message_key_frame_id);
        loadParam("radiation_unit", radiation_unit);

        // Source prediction
        loadParam("gp_mean_factor", gp_mean_factor);
        loadParam("gp_min_source_strength", gp_min_source_strength);

        // Gaussian Process
        // 2D model
        loadParam("gp_on_start_up", gp_on_start_up);
        loadParam("gp_circle_radius", gp_circle_radius);
        loadParam("gp_min_distance_between_samples_2d", gp_min_distance_between_samples_2d);
        loadParam("gp_min_update_time_2d", gp_min_update_time_2d);
        // GridMap
        loadParam("gp_grid_map_topic", gp_grid_map_topic);
        loadParam("gp_grid_map_resolution", gp_grid_map_resolution);
        // 3D model
        loadParam("gp_local_radius", gp_local_radius);
        loadParam("gp_min_distance_between_samples_3d", gp_min_distance_between_samples_3d);
        loadParam("gp_min_update_time_3d", gp_min_update_time_3d);
        // PointCloud3D
        loadParam("gp_distance_cutoff_levels", gp_distance_cutoff_levels);
        loadParam("gp_point_cloud_3d_size_levels", gp_point_cloud_3d_size_levels);

        // Least Squares
        loadParam("ls_on_start_up", ls_on_start_up);
        loadParam("ls_circle_radius", ls_circle_radius);
        loadParam("ls_min_update_time", ls_min_update_time);
        loadParam("ls_grid_map_topic", ls_grid_map_topic);
        loadParam("ls_grid_map_resolution", ls_grid_map_resolution);
        loadParam("ls_max_queue", ls_max_queue);

        // Bayesian Inference
        loadParam("bi_on_start_up", bi_on_start_up);
        loadParam("bi_circle_radius", bi_circle_radius);
        loadParam("bi_min_update_time", bi_min_update_time);
        loadParam("bi_grid_map_topic", bi_grid_map_topic);
        loadParam("bi_grid_map_resolution", bi_grid_map_resolution);

        // Field Propagation
        loadParam("fp_on_start_up", fp_on_start_up);
        loadParam("fp_circle_radius", fp_circle_radius);
        loadParam("fp_min_update_time", fp_min_update_time);
        loadParam("fp_grid_map_topic", fp_grid_map_topic);
        loadParam("fp_grid_map_resolution", fp_grid_map_resolution);

        // Set use_dose_rate to true, if message_key_rate is set
        use_dose_rate = !message_key_rate.empty();
    }

    Parameters(const Parameters &) = delete;
    Parameters &operator=(const Parameters &) = delete;
};

#endif //RADIATION_MAPPING_PARAMETERS_H