#ifndef HECTOR_RADIATION_MAPPING_EXPLORATION_SERVICE_H
#define HECTOR_RADIATION_MAPPING_EXPLORATION_SERVICE_H

#include "pch.h"

typedef actionlib::SimpleActionClient<move_base_lite_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<move_base_lite_msgs::ExploreAction> ExploreClient;

class ExplorationService {
public:
    enum RESULT {
        SUCCESS = 1,
        ABORTED = 2,
        NONE = 0
    };

    /**
     * Returns the class instance.
     * @return class instance.
     */
    static ExplorationService &instance() {
        static ExplorationService instance;
        return instance;
    }

    /**
     * Resets.
     */
    void reset();

    void moveBase(Vector2d pos);
    void moveBase(Vector2d pos, double orientation);
    void explore();
    void cancel();

    RESULT getLatestResult();
private:

    ExplorationService();
    ExplorationService(const ExplorationService &) = delete;
    ExplorationService &operator=(const ExplorationService &) = delete;

    void moveBaseDoneCb(const actionlib::SimpleClientGoalState& state, const move_base_lite_msgs::MoveBaseResultConstPtr& result);
    void moveBaseActiveCb();
    void moveBaseFeedbackCb(const move_base_lite_msgs::MoveBaseFeedbackConstPtr& feedback);

    void exploreDoneCb(const actionlib::SimpleClientGoalState& state, const move_base_lite_msgs::ExploreResultConstPtr& result);
    void exploreActiveCb();
    void exploreFeedbackCb(const move_base_lite_msgs::ExploreFeedbackConstPtr& feedback);

    RESULT reinterpretResult(const move_base_lite_msgs::ErrorCodes& result);

    MoveBaseClient move_base_client_;
    ExploreClient explore_client_;

    RESULT latest_result_;
};


#endif //HECTOR_RADIATION_MAPPING_EXPLORATION_SERVICE_H
