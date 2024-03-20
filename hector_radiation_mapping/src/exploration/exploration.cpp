#include "exploration/exploration.h"
#include "util/parameters.h"
#include "hector_radiation_mapping/sampleManager.h"
#include "exploration/exploration_map.h"
#include "exploration/exploration_service.h"
#include "util/dddynamic_reconfigure.h"

#include "models/least_squares/least_squares.h"

Exploration::Exploration() {
    ExplorationMap::instance();
    ExplorationService::instance();

    state_ = STATE::USE_METHOD;
    automated_ = Parameters::instance().exploration_automated;
    DDDynamicReconfigure::instance().registerVariable<bool>("automated",
                                                            automated_,
                                                            boost::bind(&Exploration::automatedCallback, this, _1),
                                                            "min/max",
                                                            true, false, "Exploration");

    explore();
}

void Exploration::reset() {
    ExplorationMap::instance().reset();
    ExplorationService::instance().reset();
}

void Exploration::explore() {
    if (!automated_) {
        return;
    }

    if (state_ == STATE::IDLE) {
        return;
    }
    else if (state_ == STATE::USE_EXPLORE) {
        ExplorationService::instance().explore();
        return;
    }
    else if (state_ == STATE::USE_METHOD) {
        ExplorationService::RESULT result = ExplorationService::instance().getLatestResult();

        // get latest model results
        LeastSquares::Result ls_result = LeastSquares::instance().getResult();

        // use exploration map to determine next goal
        Vector2d goal = ExplorationMap::instance().getClosestLocation(ls_result.radius_minima.back());

        // move to goal
        ExplorationService::instance().moveBase(ls_result.radius_minima.back());
    }
}

void Exploration::automatedCallback(bool automated) {
    automated_ = automated;
    if (automated) {
        explore();
    } else {
        ExplorationService::instance().cancel();
    }
}