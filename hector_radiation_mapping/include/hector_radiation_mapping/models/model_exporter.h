#ifndef RADIATION_MAPPING_MODEL_EXPORTER_H
#define RADIATION_MAPPING_MODEL_EXPORTER_H

#include "hector_radiation_mapping_msgs/ExportService.h"
#include "hector_radiation_mapping_msgs/ExportServiceRequest.h"
#include "hector_radiation_mapping_msgs/ExportServiceResponse.h"
#include "custom_geotiff/geotiff_writer.h"

class Source;
class PointCloud3D;

/**
 * @brief The ModelExporter class
 * This class is used to export the 2D map as geotiff file and the 3D map as point cloud.
 */
class ModelExporter {
public:
    static ModelExporter &instance();

    /**
     * Computes the error of a source compared to an assumed value.
     * @param sourceId
     * @param sourcePos
     * @param slamGrid
     * @param radius
     * @param actualValue
     * @param numSamples
     * @param wallsOn
     * @param useGlobal
     * @return The error.
     */
    double computeSourceError2D(int sourceId, const Eigen::Vector3d &sourcePos, nav_msgs::OccupancyGrid &slamGrid,
                              double radius, double actualValue, int numSamples, bool wallsOn, bool useGlobal);

private:
    /**
     * Private constructor for singleton pattern.
     * It initializes the export service.
     */
    ModelExporter();

    ModelExporter(const ModelExporter &) = delete;

    ModelExporter &operator=(const ModelExporter &) = delete;

    /**
     * Callback for the export service. It exports the 2D and 3D map.
     * @param req The request.
     * @param res The response.
     * @return True.
     */
    bool exportServiceCallback(hector_radiation_mapping_msgs::ExportService::Request &req,
                               hector_radiation_mapping_msgs::ExportService::Response &res);

    /**
     * Exports the 2D map.
     */
    void export2DMap(std::string path);

    /**
     * Exports the 3D map.
     */
    void export3DMap(std::string path);

    /**
     * Get all grid maps, paired with their corresponding occupancy grid.
     * @return All grid maps, paired with their corresponding occupancy grid.
     */
    std::vector<std::pair<grid_map::GridMap, std::shared_ptr<nav_msgs::OccupancyGrid>>> getMaps();

    /**
     * Get all sources.
     * @return All sources.
     */
    std::vector<std::shared_ptr<Source>> getSources();

    /**
     * Get the trajectory.
     * @return The trajectory.
     */
    std::vector<Eigen::Vector2f> getTrajectory();

    /**
     * Set up the combined grid for slam and grid map.
     * @param slamGrid
     * @return The combined grid.
     */
    nav_msgs::OccupancyGrid setUpCombinedGrid(nav_msgs::OccupancyGrid &slamGrid);

    /**
     * Create a geotiff file
     * @param fileName The name of the file
     * @param path The path of the file
     * @param data Grid map data as vector
     * @param sources All sources to be included in the geotiff
     * @param trajectory The trajectory to be included in the geotiff
     * @param slamGrid The slam occupancy grid
     * @param combinedGrid The combined occupancy grid
     * @param useLogScale Whether to use a log scale or not
     */
    void createGeoTiff(const std::string &fileName, const std::string &path, Vector &data,
                       std::vector<std::shared_ptr<Source>> &sources,
                       std::vector<Eigen::Vector2f> &trajectory, nav_msgs::OccupancyGrid &slamGrid,
                       nav_msgs::OccupancyGrid &combinedGrid, bool useLogScale);


    volatile bool exporting_;
    std::shared_ptr<geotiff::GeotiffWriter> geotiff_writer_;
    ros::ServiceServer exportService_;
};

#endif //RADIATION_MAPPING_MODEL_EXPORTER_H
