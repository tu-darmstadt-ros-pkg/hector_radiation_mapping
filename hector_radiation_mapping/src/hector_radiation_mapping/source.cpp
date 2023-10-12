#include "hector_radiation_mapping/source.h"
#include "hector_radiation_mapping/sampleManager.h"
#include "util/parameters.h"

Source::Source(const Eigen::Vector3d &position, double cps, double doseRate, bool createMarker) {
    ROS_INFO_STREAM("Source 0 " << cps << "|" << doseRate);
    id_ = idCounter_;
    idCounter_++;
    position_ = position;
    cps_ = cps + SampleManager::instance().getBackgroundRadiationCps();
    doseRate_ = doseRate + SampleManager::instance().getBackgroundRadiationDoseRate();
    confirmed_ = false;
    name_ = std::to_string(id_);
    if (Parameters::instance().useDoseRate) {
        description_ = "(" + name_ + ") " + std::to_string(doseRate_);
    } else {
        description_ = "(" + name_ + ") " + std::to_string(cps_);
    }
    if (createMarker) {
        marker_ = std::make_shared<TextMarker>(position, description_);
    }
}

Source::~Source(){
    if (marker_) {
        marker_->deleteMarker();
    }
}

void Source::setConfirmed(bool confirmed) {
    this->confirmed_ = confirmed;
}

double Source::getStrength() const {
    return Parameters::instance().useDoseRate ? doseRate_ : cps_;
}