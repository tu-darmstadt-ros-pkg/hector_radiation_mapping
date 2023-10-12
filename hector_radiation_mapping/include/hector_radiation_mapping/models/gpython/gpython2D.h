#ifndef RADIATION_MAPPING_GPYTHON2D_H
#define RADIATION_MAPPING_GPYTHON2D_H

#include "gpython.h"
#include "maps/gridmap.h"
#include "hector_radiation_mapping/source_interactive.h"

/**
 * @brief The GPython2D class is a singleton and manages the 2D GridMap, the evaluation process of its positions in the
 * GPython model and the localisation of the sources. Runs in a separate thread.
 */
class GPython2D {
public:
    /**
     * Returns the instance of the GPython2D class.
     * @return The instance of the GPython2D class.
     */
    static GPython2D &instance();

    /**
     * Returns if the GPython2D class is active.
     * @return True if the GPython2D class is active.
     */
    bool isActive() { return active_; };

    /**
     * Deactivates the GPython2D class.
     */
    void deactivate();

    /**
     * Activates the GPython2D class.
     */
    void activate();

    /**
     * Shuts down the GPython2D class.
     */
    void shutDown();

    /**
     * Returns the grid map.
     * @return The grid map.
     */
    std::shared_ptr<GridMap> getGridMap() { return gridMap_; };

    /**
     * Get the sources.
     * @return The sources.
     */
    std::vector<std::shared_ptr<SourceInteractive>> getSources() { return sources_; };

    /**
     * Check if the GPython2D class has a slam map.
     * @return True if the GPython2D class has a slam map.
     */
    bool hasSlamMap() { return slamMap_ != nullptr; };

    /**
     * Get the slam map.
     * @return The slam map.
     */
    std::shared_ptr<nav_msgs::OccupancyGrid> getSlamMap() { return slamMap_; };

    /**
     * Set whether to use evaluate the gridmap in a circle with certain diameter or evaluate the whole map.
     * @param useCircle
     */
    void setUseCircle(bool useCircle);

    /**
     * Get the grid map and the slam map as a pair.
     * @return The grid map and the slam map as a pair.
     */
    std::pair<grid_map::GridMap, std::shared_ptr<nav_msgs::OccupancyGrid>> getExportGridMap();

    /**
     * Forcefully trigger the evaluation of the grid map.
     */
    void triggerEvaluation();

private:
    GPython2D();
    GPython2D(const GPython2D &) = delete;
    GPython2D &operator=(const GPython2D &) = delete;

    /**
     * The update loop of the GPython2D class. It waits for the activation signal and then evaluates the model and
     * updates the grid map.
     */
    void updateLoop();

    /**
     * Updates the grid map with the given mean and standard deviation.
     * @param mean
     * @param std_dev
     * @param useCircle
     * @param center
     */
    void updateMap(const Vector &mean, const Vector &std_dev, bool useCircle = false,
                   const Vector2d &center = Vector2d(0, 0));

    /**
     * Get the positions of the grid map to evaluate.
     * @param slamMap
     * @param useCircle
     * @param center
     * @return The positions of the grid map to evaluate.
     */
    Matrix getSamplePositions(const std::shared_ptr<nav_msgs::OccupancyGrid> &slamMap, bool useCircle = false,
                              const Vector2d &center = Vector2d(0, 0));

    /**
     * Update the prediction of the sources.
     * @param positions
     * @param mean
     */
    void updateSourcePrediction(const Matrix &positions, const Vector &mean);

    /**
     * Compute the gradient step for the given position. Used in updateSourcePrediction().
     * @param position
     * @return The gradient step for the given position.
     */
    Vector computeGradientStep(const Vector &position);

    /**
     * Set the minimum update time for the grid map, to limit data traffic.
     * @param time
     */
    void setMinUpdateTime(int time);

    /**
     * Callback for the slam map.
     * @param gridMsgPtr
     */
    void slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &gridMsgPtr);

    /**
     * Confirm a source by its id.
     * @param sourceId
     */
    void confirmSource(int sourceId);

    /**
     * Callback for the interactive marker feedback of the interactive markers of the sources. It confirms the source if
     * the feedback is a button click.
     * @param feedback
     */
    void interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    std::shared_ptr<GridMap> gridMap_;
    std::string layerNameStdDev_;
    std::string layerNameMean_;
    std::shared_ptr<nav_msgs::OccupancyGrid> slamMap_;
    std::vector<std::shared_ptr<SourceInteractive>> sources_;
    volatile bool useCircle_;
    volatile int minUpdateTime_;
    std::string groupName_;
    std::shared_ptr<ros::Subscriber> slamMapSubscriber_;

    // Threading
    volatile bool active_;
    volatile bool doEvaluation_;
    std::thread updateThread_;
    std::condition_variable waitCondition_;
    std::mutex activation_mtx_;
};

#endif //RADIATION_MAPPING_GPYTHON2D_H
