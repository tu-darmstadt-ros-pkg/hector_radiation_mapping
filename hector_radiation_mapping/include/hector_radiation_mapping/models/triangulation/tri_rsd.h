#ifndef HECTOR_RADIATION_MAPPING_TRI_RSD_H
#define HECTOR_RADIATION_MAPPING_TRI_RSD_H

#include "pch.h"
#include "maps/gridmap.h"
#include "marker/marker.h"
#include "models/model.h"
#include "hector_radiation_mapping/sample.h"
#include "hector_radiation_mapping/source.h"

class TriRSD : public Model{
public:
    /**
     * Returns the class instance.
     * @return class instance.
     */
    static TriRSD &instance() {
        static TriRSD instance;
        return instance;
    }

    /**
     * Resets the model.
     */
    void reset() override;

private:
    TriRSD();
    TriRSD(const TriRSD &) = delete;
    TriRSD &operator=(const TriRSD &) = delete;

    void update() override;

    void evaluate();

    /**
     * Callback for the slam map.
     * @param gridMsgPtr
     */
    void slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &gridMsgPtr);


    /// Thread pipeline
    volatile bool useCircle_;

    std::string layerNameVal_;
    std::string layerNameErr_;

    std::shared_ptr<GridMap> grid_map_;
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map_;
    std::shared_ptr<ros::Subscriber> slam_map_subscriber_;

    std::vector<TextMarker> text_markers_;
};

#endif //HECTOR_RADIATION_MAPPING_TRI_RSD_H
