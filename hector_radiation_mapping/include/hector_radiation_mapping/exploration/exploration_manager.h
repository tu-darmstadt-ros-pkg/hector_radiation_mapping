#ifndef HECTOR_RADIATION_MAPPING_EXPLORATION_MANAGER_H
#define HECTOR_RADIATION_MAPPING_EXPLORATION_MANAGER_H

#include "pch.h"
#include "marker/marker.h"

/**
 * @brief The ExplorationManager class
 */
class ExplorationManager {
public:
    enum STATE {
        NONE = -1,
        IDLE = 0,
        EXPLORE = 1,
        INSPECTION = 2,
        CLOSE_INSPECTION = 3
    };

    struct Goal {
        STATE state_;
        Vector2d position_;
        double orientation_;
    };

    /**
     * Returns the class instance.
     * @return class instance.
     */
    static ExplorationManager& instance() {
        static ExplorationManager instance;
        return instance;
    }

    /**
     * Resets.
     */
    void reset();

    void shutdown();


private:
    ExplorationManager();

    ExplorationManager(const ExplorationManager &) = delete;

    ExplorationManager &operator=(const ExplorationManager &) = delete;

    void updateLoop();

    void explore();

    void automatedCallback(bool automated);

    void timeoutCallback(double timeout);

    std::vector<TextMarker> text_markers_;
    std::atomic<bool> automated_{};
    std::atomic<double> timeout_{};
    std::thread update_thread_;

    STATE state_{};
    std::queue<Vector2d> inspection_queue_;
    Goal last_goal_;
};

#endif //HECTOR_RADIATION_MAPPING_EXPLORATION_MANAGER_H
