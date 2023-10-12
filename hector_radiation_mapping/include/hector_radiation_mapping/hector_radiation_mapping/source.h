#ifndef RADIATION_MAPPING_SOURCE_H
#define RADIATION_MAPPING_SOURCE_H

#include "marker/marker.h"

const double cpsToMikroSievertPerHour_ = (1.0 / 3.0) * ((79.76 / 18372.0) + (8.36 / 1974.0) + (191.01 / 43815.0));
const double mikroSievertPerHourToCps_ = 1 / cpsToMikroSievertPerHour_;

/**
 * @brief The Source class
 * This class represents a source of radiation. It contains the position, the cps and the dose rate of the source.
 * It also contains a marker that is used to visualize the source in rviz.
 */

class Source {
public:
    Source(const Eigen::Vector3d &position, double cps, double doseRate, bool createMarker = true);

    ~Source();

    /**
     * Check if the source is confirmed
     * @return true if the source is confirmed
     */
    bool isConfirmed() const { return confirmed_; };

    /**
     * Get the dose rate of the source
     * @return dose rate of the source
     */
    double getDoseRate() const { return doseRate_; };

    /**
     * Get the cps of the source
     * @return cps of the source
     */
    double getCps() const { return cps_; };

    /**
     * Get the strength of the source. Either cps or dose rate depending on the parameter useDoseRate which can be
     * configured in the config file
     * @return strength of the source
     */
    double getStrength() const;

    /**
     * Get the id of the source
     * @return id of the source
     */
    int getId() const { return id_; };

    /**
     * Get the position of the source
     * @return position of the source
     */
    Eigen::Vector3d getPos() const { return position_; };

    /**
     * Set whether the source is confirmed or not
     * @param confirmed
     */
    virtual void setConfirmed(bool confirmed);

protected:
    inline static int idCounter_ = 0;

    int id_;
    bool confirmed_;
    double cps_;
    double doseRate_;
    std::string name_;
    std::string description_;
    Eigen::Vector3d position_;
    std::shared_ptr<TextMarker> marker_;
};

#endif  // RADIATION_MAPPING_SOURCE_H
