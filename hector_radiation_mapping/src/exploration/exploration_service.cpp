#include "exploration/exploration_service.h"
#include "util/parameters.h"
#include "exploration/exploration.h"

ExplorationService::ExplorationService() :
        move_base_client_("/move_base", true),
        explore_client_("/explore", true) {
    move_base_client_.waitForServer();
    explore_client_.waitForServer();
    latest_result_ = RESULT::NONE;
    STREAM("Exploration clients initialized.");
}

void ExplorationService::reset() {
    move_base_client_.cancelAllGoals();
    explore_client_.cancelAllGoals();
}

void ExplorationService::moveBase(Vector2d pos) {
    move_base_lite_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = Parameters::instance().world_frame; // world?
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = pos.x();
    goal.target_pose.pose.position.y = pos.y();

    goal.follow_path_options.goal_pose_angle_tolerance = M_PI;
    goal.follow_path_options.goal_pose_position_tolerance = 0.2;
    goal.follow_path_options.use_path_orientation = true;
    goal.follow_path_options.reverse_allowed = true;

    move_base_client_.sendGoal(goal, boost::bind(&ExplorationService::moveBaseDoneCb, this, _1, _2),
                               boost::bind(&ExplorationService::moveBaseActiveCb, this),
                               boost::bind(&ExplorationService::moveBaseFeedbackCb, this, _1));
    STREAM("Move base goal sent.");
}

void ExplorationService::moveBase(Vector2d pos, double orientation) {

}

void ExplorationService::explore() {
    move_base_lite_msgs::ExploreGoal goal;
    goal.reset_stuck_history = true;
    goal.desired_speed = 0.5;
    explore_client_.sendGoal(goal, boost::bind(&ExplorationService::exploreDoneCb, this, _1, _2),
                             boost::bind(&ExplorationService::exploreActiveCb, this),
                             boost::bind(&ExplorationService::exploreFeedbackCb, this, _1));
    STREAM("Explore goal sent.");
}

void ExplorationService::moveBaseDoneCb(const actionlib::SimpleClientGoalState &state,
                                 const move_base_lite_msgs::MoveBaseResultConstPtr &result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    move_base_lite_msgs::ErrorCodes error;
    latest_result_ = reinterpretResult(result->result);
    Exploration::instance().explore();
}

void ExplorationService::moveBaseActiveCb() {
    ROS_INFO("MoveBase action active");
}

void ExplorationService::moveBaseFeedbackCb(const move_base_lite_msgs::MoveBaseFeedbackConstPtr &feedback) {
    ROS_INFO("Got Feedback: %f", feedback->percent_complete);
}

void ExplorationService::exploreDoneCb(const actionlib::SimpleClientGoalState &state,
                                const move_base_lite_msgs::ExploreResultConstPtr &result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    latest_result_ = reinterpretResult(result->result);
    Exploration::instance().explore();
}

void ExplorationService::exploreActiveCb() {
    ROS_INFO("Explore action active");
}

void ExplorationService::exploreFeedbackCb(const move_base_lite_msgs::ExploreFeedbackConstPtr &feedback) {

}

ExplorationService::RESULT ExplorationService::reinterpretResult(const move_base_lite_msgs::ErrorCodes &result) {
    switch (result.val) {
        // overall behavior
        case move_base_lite_msgs::ErrorCodes::Type::SUCCESS: // 1
            return RESULT::SUCCESS;
        case move_base_lite_msgs::ErrorCodes::Type::FAILURE: // 99999
            return RESULT::ABORTED;
            // Planner failed to generate plan
        case move_base_lite_msgs::ErrorCodes::Type::PLANNING_FAILED: // -1
            return RESULT::ABORTED;
            // Plan generated, but not valid
        case move_base_lite_msgs::ErrorCodes::Type::INVALID_MOTION_PLAN: // -2
            return RESULT::ABORTED;
            // Unable to follow path within thresholds
        case move_base_lite_msgs::ErrorCodes::Type::CONTROL_FAILED: // -4
            return RESULT::ABORTED;
            // Preempted before reaching final state
        case move_base_lite_msgs::ErrorCodes::Type::PREEMPTED: // -7
            return RESULT::ABORTED;
            // Stuck detection triggered
        case move_base_lite_msgs::ErrorCodes::Type::STUCK_DETECTED: // -8
            return RESULT::ABORTED;
            // Tf failure
        case move_base_lite_msgs::ErrorCodes::Type::TF_LOOKUP_FAILURE: // -9
            return RESULT::ABORTED;
            // planning & kinematics request errors
        case move_base_lite_msgs::ErrorCodes::Type::START_STATE_IN_COLLISION: // -10
            return RESULT::ABORTED;
        case move_base_lite_msgs::ErrorCodes::Type::GOAL_IN_COLLISION: // -12
            return RESULT::ABORTED;
        default:
            return RESULT::ABORTED;
    }
}

void ExplorationService::cancel() {
    move_base_client_.cancelAllGoals();
    explore_client_.cancelAllGoals();
}

ExplorationService::RESULT ExplorationService::getLatestResult() {
    return latest_result_;
}
