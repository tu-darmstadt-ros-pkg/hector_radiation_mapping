#ifndef HECTOR_RADIATION_MAPPING_TRIANGULATION_H
#define HECTOR_RADIATION_MAPPING_TRIANGULATION_H

#include "pch.h"
#include "maps/gridmap.h"
#include "marker/marker.h"
#include "models/model.h"
#include "hector_radiation_mapping/sample.h"
#include "hector_radiation_mapping/source.h"

class Triangulation : public Model{
public:
    /**
     * Returns the class instance.
     * @return class instance.
     */
    static Triangulation &instance() {
        static Triangulation instance;
        return instance;
    }

    /**
     * Resets the model.
     */
    void reset() override;

    void useErrorMap(bool use);
    static double error(const Sample &sample1, const Sample &sample2, const Eigen::Vector3d &posSource);

private:
    Triangulation();
    Triangulation(const Triangulation &) = delete;
    Triangulation &operator=(const Triangulation &) = delete;

    void update() override;
    void updateSourcePrediction();
    void update2DView();

    Vector sample2DMapError(const Eigen::MatrixX2d &positions);
    Vector sample2DMapValue(const Eigen::MatrixX2d &positions);
    Eigen::Vector3d nelderMeadMinimization(Eigen::Vector3d startPos);

    double calculateSourceValue(const Eigen::Vector3d &position);
    static double calculateWeight();
    double calculateIntensity(const Eigen::Vector3d &position);
    static double calculateIntensity(const Eigen::Vector3d &position, const Source &source);

    /**
     * Callback for the slam map.
     * @param gridMsgPtr
     */
    void slamMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &gridMsgPtr);


    /// Thread pipeline
    volatile bool useCircle_;
    volatile bool useExpensiveErrorMap_;

    ///
    double zLevel2D_;
    double zLevel2Dtemp_;
    std::vector<Source> sources_;


    std::string layerNameVal_;
    std::string layerNameErr_;

    std::shared_ptr<GridMap> grid_map_;
    std::shared_ptr<nav_msgs::OccupancyGrid> slam_map_;
    std::shared_ptr<ros::Subscriber> slam_map_subscriber_;

};

#endif //HECTOR_RADIATION_MAPPING_TRIANGULATION_H
