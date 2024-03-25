#ifndef RADIATION_MAPPING_SAMPLE_H
#define RADIATION_MAPPING_SAMPLE_H

#include <utility>

#include "pch.h"

/**
 * @brief The Sample class
 * This class represents a sample of radiation. It contains the position, the cps and the dose rate of the sample.
 * It also contains a timestamp. The id is used to identify the sample.
 */
struct Sample{
    Sample(Vector3d sensor_position, Vector3d base_position, double cps, double doseRate, ros::Time time){
        this->sensor_position_ = std::move(sensor_position);
        this->base_position_ = std::move(base_position);
        this->cps_ = cps;
        this->dose_rate_ = doseRate;
        this->time_ = time;
        this->id_ = id_counter_++;
        this->effective_radius_ = std::sqrt(dose_rate_ / 0.01);
    }

    Sample(){};

    /**
     * Get the 2D position of the sample
     * @return 2D position of the sample
     */
    Vector2d get2DPos() const {return {sensor_position_.x(), sensor_position_.y()};};

    Vector2d getBase2DPos() const {return {base_position_.x(), base_position_.y()};};

    /**
     * Get the 3D position of the sample with z = 0
     * @return 3D position of the sample with z = 0
     */
    Vector3d get3DzZero(){return {sensor_position_.x(), sensor_position_.y(), 0.0};};

    Vector3d sensor_position_;
    Vector3d base_position_;
    double cps_;
    double dose_rate_;
    ros::Time time_;
    int id_;
    double effective_radius_;

private:
    inline static int id_counter_ = 0;
};

#endif //RADIATION_MAPPING_SAMPLE_H
