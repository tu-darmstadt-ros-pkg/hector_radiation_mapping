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
     * Returns the class instance.
     * @return class instance.
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
    std::shared_ptr<GridMap> getGridMap() { return grid_map_; };

    /**
     * Get the sources.
     * @return The sources.
     */
    std::vector<std::shared_ptr<SourceInteractive>> getSources() { return sources_; };

    /**
     * Check if the GPython2D class has a slam map.
     * @return True if the GPython2D class has a slam map.
     */
    bool hasSlamMap() { return slam_map_ != nullptr; };

    /**
     * Get the slam map.
     * @return The slam map.
     */
    std::shared_ptr<nav_msgs::OccupancyGrid> getSlamMap() { return slam_map_; };

    /**
     * Set whether to use evaluate the gridmap in a circle with certain diameter or evaluate the whole map.
     * @param use_circle
     */
    void setUseCircle(bool use_circle);

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
     * @param use_circle
     * @param center
     */
    void updateMap(const Vector &mean, const Vector &std_dev, bool use_circle = false,
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
     * @param grid_msg_ptr
     */
    void slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &grid_msg_ptr);

    /**
     * Confirm a source by its id.
     * @param source_id
     */
    void confirmSource(int source_id);

    /**
     * Callback for the interactive marker feedback of the interactive markers of the sources. It confirms the source if
     * the feedback is a button click.
     * @param feedback
     */
    void interactiveMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    std::shared_ptr<GridMap> grid_map_;
    std::string layer_name_std_dev_;
    std::string layer_name_mean_;
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map_;
    std::vector<std::shared_ptr<SourceInteractive>> sources_;
    volatile bool use_circle_;
    volatile int min_update_time_;
    std::string group_name_;
    std::shared_ptr<ros::Subscriber> slam_map_subscriber_;

    // Threading
    volatile bool active_;
    volatile bool do_evaluation_;
    std::thread update_thread_;
    std::condition_variable update_condition_;
    std::mutex activation_mtx_;
};

#endif //RADIATION_MAPPING_GPYTHON2D_H
