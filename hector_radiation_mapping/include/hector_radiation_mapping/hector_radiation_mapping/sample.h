#ifndef RADIATION_MAPPING_SAMPLE_H
#define RADIATION_MAPPING_SAMPLE_H

/**
 * @brief The Sample class
 * This class represents a sample of radiation. It contains the position, the cps and the dose rate of the sample.
 * It also contains a timestamp. The id is used to identify the sample.
 */
struct Sample{
    Sample(Eigen::Vector3d position, double cps, double doseRate, ros::Time time){
        this->position_ = position;
        this->cps_ = cps;
        this->doseRate_ = doseRate;
        this->time_ = time;
        this->id_ = idCounter_++;
    }

    Sample(){};

    /**
     * Get the 2D position of the sample
     * @return 2D position of the sample
     */
    Eigen::Vector2d get2DPos(){return {position_.x(), position_.y()};};

    /**
     * Get the 3D position of the sample with z = 0
     * @return 3D position of the sample with z = 0
     */
    Eigen::Vector3d get3DzZero(){return {position_.x(), position_.y(), 0.0};};

    Eigen::Vector3d position_;
    double cps_;
    double doseRate_;
    ros::Time time_;
    int id_;

private:
    inline static int idCounter_ = 0;
};

#endif //RADIATION_MAPPING_SAMPLE_H
