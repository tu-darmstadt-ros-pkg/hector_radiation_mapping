#ifndef HECTOR_RADIATION_MAPPING_EXPLORATION_H
#define HECTOR_RADIATION_MAPPING_EXPLORATION_H

#include "pch.h"
#include "marker/marker.h"

/**
 * @brief The Exploration class
 */
class Exploration {
public:
    enum STATE{
        IDLE = 0,
        USE_EXPLORE = 1,
        USE_METHOD = 2,
        CLOSE_INSPECTION = 3
    };

    /**
     * Returns the class instance.
     * @return class instance.
     */
    static Exploration &instance() {
        static Exploration instance;
        return instance;
    }

    /**
     * Resets.
     */
    void reset();

    void explore();

private:
    Exploration();
    Exploration(const Exploration &) = delete;
    Exploration &operator=(const Exploration &) = delete;


    void automatedCallback(bool automated);

    std::vector<TextMarker> text_markers_;
    std::atomic_bool automated_{};
    STATE state_{};
};

#endif //HECTOR_RADIATION_MAPPING_EXPLORATION_H
