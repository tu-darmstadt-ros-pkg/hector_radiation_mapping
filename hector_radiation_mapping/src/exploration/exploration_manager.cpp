#include "exploration/exploration_manager.h"
#include "util/parameters.h"
#include "hector_radiation_mapping/sampleManager.h"
#include "exploration/exploration_map.h"
#include "exploration/exploration_service.h"
#include "util/dddynamic_reconfigure.h"
#include "models/least_squares/least_squares.h"

ExplorationManager::ExplorationManager() {
    ExplorationMap::instance();
    ExplorationService::instance();

    state_ = STATE::INSPECTION;
    automated_ = Parameters::instance().exploration_automated;
    timeout_ = Parameters::instance().exploration_timeout;
    last_goal_ = Goal{STATE::NONE, Vector2d(0.0, 0.0), 0.0};

    DDDynamicReconfigure::instance().registerVariable<bool>("automated",
                                                            automated_,
                                                            boost::bind(&ExplorationManager::automatedCallback, this,
                                                                        _1),
                                                            "min/max",
                                                            true, false, "ExplorationManager");
    DDDynamicReconfigure::instance().registerVariable<double>("timeout",
                                                              timeout_,
                                                              boost::bind(&ExplorationManager::timeoutCallback, this,
                                                                          _1),
                                                              "min/max",
                                                              0.0, 10.0, "ExplorationManager");

    update_thread_ = std::thread(&ExplorationManager::updateLoop, this);
}

void ExplorationManager::shutdown() {
    ExplorationMap::instance().shutdown();
    ExplorationService::instance().shutdown();
}

void ExplorationManager::reset() {
    ExplorationMap::instance().reset();
    ExplorationService::instance().reset();
    state_ = STATE::INSPECTION;
    automated_ = Parameters::instance().exploration_automated;
    timeout_ = Parameters::instance().exploration_timeout;
    last_goal_ = Goal{STATE::NONE, Vector2d(0.0, 0.0), 0.0};
}

void ExplorationManager::updateLoop() {
    ros::Rate rate(1);

    while (ros::ok()) {
        if (automated_) {
            ExplorationMap::instance().updateExplorationMap();
            explore();
        }
        rate.sleep();
    }
}

void ExplorationManager::explore() {
    if (!ExplorationMap::instance().hasLocations()) {
        ExplorationService::instance().explore();
        return;
    }

    // evaluate result of last move
    ExplorationService::RESULT exp_result = ExplorationService::instance().getLatestResult();
    if (exp_result == ExplorationService::RESULT::ABORTED) {
        ROS_INFO_STREAM("Exploration aborted.");
        if(last_goal_.state_ == STATE::INSPECTION || last_goal_.state_ == STATE::CLOSE_INSPECTION){
            ExplorationMap::instance().setLocationReachable(last_goal_.position_, false);
        }
    } else if (exp_result == ExplorationService::RESULT::TIMEOUT) {
        ROS_INFO_STREAM("Exploration timed out.");
        if (last_goal_.state_ == STATE::INSPECTION || last_goal_.state_ == STATE::CLOSE_INSPECTION) {

        }
    } else if (exp_result == ExplorationService::RESULT::SUCCESS) {
        ROS_INFO_STREAM("Exploration successful.");
        if (last_goal_.state_ == STATE::INSPECTION || last_goal_.state_ == STATE::CLOSE_INSPECTION) {

        }
    }

    // decide next move
    switch (state_) {
        case IDLE:
            return;
        case EXPLORE:
            ExplorationService::instance().explore();
            return;
        case INSPECTION: {
            // get latest model results
            std::shared_ptr<LeastSquares::Result> ls_result = LeastSquares::instance().getResult();
            if (ls_result == nullptr) {
                ExplorationService::instance().explore();
                return;
            }

            Vector2d goal;
            if (!ExplorationMap::instance().getClosestBaseLocation(ls_result->radius_minima.back(), goal)) {
                ExplorationService::instance().explore();
                return;
            }

            Vector3d pos3d(goal.x(), goal.y(), 0.0);
            for (TextMarker text_marker: text_markers_) {
                text_marker.deleteMarker();
            }
            TextMarker marker(pos3d, "Goal");
            text_markers_.push_back(marker);

            // move to goal
            if (ExplorationService::instance().moveBase(goal, timeout_, 0.5)) {
                last_goal_ = Goal{INSPECTION, goal, 0.0};
            } else {
                last_goal_ = Goal{INSPECTION, goal, 0.0};
            }
            return;
        }
        case CLOSE_INSPECTION:{
            // get latest model results
            std::shared_ptr<LeastSquares::Result> ls_result = LeastSquares::instance().getResult();
            if (ls_result == nullptr) {
                ExplorationService::instance().explore();
                return;
            }

            if (inspection_queue_.empty()) {
                // use exploration map to determine next goal
                Vector2d goal;
                if (!ExplorationMap::instance().getClosestSensorLocation(ls_result->radius_minima.back(), goal)) {
                    ExplorationService::instance().explore();
                    return;
                }

                Vector3d pos3d(goal.x(), goal.y(), 0.0);

                for (TextMarker text_marker: text_markers_) {
                    text_marker.deleteMarker();
                }
                TextMarker marker(pos3d, "Goal");

                // generate queue of inspection points based on goal
                //inspection_queue_ = ExplorationMap::instance().getInspectionQueue(goal);
                inspection_queue_.push(goal);
                text_markers_.push_back(marker);
            }

            // move to goal
            if (ExplorationService::instance().moveBase(inspection_queue_.front(), timeout_, 0.5)) {
                // move successful

            } else {
                // move failed or timed out

            }

            inspection_queue_.pop();
            return;
        }
        case NONE:
            break;
    }
}

void ExplorationManager::automatedCallback(bool automated) {
    automated_ = automated;
    if (automated) {
        ROS_INFO_STREAM("Automated exploration enabled.");
    } else {
        ExplorationService::instance().cancel();
        ROS_INFO_STREAM("Automated exploration disabled.");
    }
}

void ExplorationManager::timeoutCallback(double timeout) {
    timeout_ = timeout;
    ROS_INFO_STREAM("Timeout set to " << timeout << " seconds.");
}