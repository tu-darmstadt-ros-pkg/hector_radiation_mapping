#include <filesystem>
#include <pcl/io/ply_io.h>

#include "models/model_exporter.h"
#include "models/model_manager.h"
#include "models/gpython/gpython.h"
#include "models/gpython/gpython3D.h"
#include "models/gpython/gpython2D.h"
#include "maps/pointcloud3d.h"
#include "util/color.h"
#include "util/parameters.h"
#include "util/util.h"
#include "hector_radiation_mapping/source.h"
#include "hector_radiation_mapping/sampleManager.h"



ModelExporter::ModelExporter() {
    exporting_ = false;
    geotiff_writer_ = std::make_shared<geotiff::GeotiffWriter>();
    exportService_ = ros::ServiceServer(
            Parameters::instance().nodeHandle_->advertiseService("exportModel", &ModelExporter::exportServiceCallback,
                                                                 this));
}

ModelExporter &ModelExporter::instance() {
    static ModelExporter instance;
    return instance;
}

bool ModelExporter::exportServiceCallback(hector_radiation_mapping_msgs::ExportService::Request &req,
                                          hector_radiation_mapping_msgs::ExportService::Response &res) {
    if (exporting_) {
        STREAM("Already exporting!");
        return false;
    }
    std::string path = req.path;
    exporting_ = true;
    STREAM("Exporting to \"" << path << "\".");
    export3DMap(path); // needs to before export2DMap because the sources get generated during export3DMap
    export2DMap(path);
    exporting_ = false;
    STREAM("Export finished!");
    res.success = true;
    return true;
}

void ModelExporter::export2DMap(std::string path) {
    if(path.empty()) {
        path = Util::getExportPath("maps");
    }
    std::vector<Eigen::Vector2f> trajectory = getTrajectory();
    std::vector<std::pair<grid_map::GridMap, std::shared_ptr<nav_msgs::OccupancyGrid>>> maps = getMaps();
    std::vector<std::shared_ptr<Source>> sources = getSources();

    for (std::pair<grid_map::GridMap, std::shared_ptr<nav_msgs::OccupancyGrid>> &map_pair: maps) {
        grid_map::GridMap &gridMap = map_pair.first;
        std::shared_ptr<nav_msgs::OccupancyGrid> slamGrid = map_pair.second;
        nav_msgs::OccupancyGrid combinedGrid(setUpCombinedGrid(*slamGrid));
        std::vector<std::string> layers = gridMap.getLayers();
        int gridWidth = combinedGrid.info.width;
        int gridHeight = combinedGrid.info.height;

        for (const std::string &layerName: layers) {
            STREAM_DEBUG("LayerName: " << layerName);

            Vector finalData(gridWidth * gridHeight);
            grid_map::Matrix &mapData = (gridMap)[layerName];
            double min = std::max(0.004f, mapData.minCoeff());

            STREAM_DEBUG("Interpolate gridmap!");
            for (int i = 0; i < gridWidth * gridHeight; i++) {
                grid_map::Position position;
                position.x() = (i % gridWidth) * combinedGrid.info.resolution + combinedGrid.info.origin.position.x;
                position.y() = (i / gridWidth) * combinedGrid.info.resolution + combinedGrid.info.origin.position.y;

                if (gridMap.isInside(position)) {
                    float pred = gridMap.atPosition(layerName, position, grid_map::InterpolationMethods::INTER_LINEAR);
                    finalData[i] = std::max(0.004f, pred);
                } else {
                    grid_map::Position closestPosition = gridMap.getClosestPositionInMap(position);
                    try {
                        float value = gridMap.atPosition(layerName, closestPosition);
                        finalData[i] = std::max(0.004f, value);
                    } catch (std::out_of_range &exception) {
                        finalData[i] = min;
                        continue;
                    }
                }
            }
            if (layerName == "confidence") {
                createGeoTiff(layerName, path, finalData, sources, trajectory, *slamGrid, combinedGrid, false);
            } else if (layerName == "prediction") {
                if (Parameters::instance().useDoseRate) {
                    finalData = finalData.array() + SampleManager::instance().getBackgroundRadiationDoseRate();
                } else {
                    finalData = (finalData * cpsToMikroSievertPerHour_).array() +
                                SampleManager::instance().getBackgroundRadiationDoseRate();
                }

                createGeoTiff(layerName + "_logp1", path, finalData, sources, trajectory, *slamGrid, combinedGrid,
                              true);
                createGeoTiff(layerName, path, finalData, sources, trajectory, *slamGrid, combinedGrid, false);
            } else {
                createGeoTiff(layerName, path, finalData, sources, trajectory, *slamGrid, combinedGrid, false);
            }
        }
    }
}

void ModelExporter::export3DMap(std::string path) {
    STREAM_DEBUG("ModelExport export3DMap()");
    pcl::PointCloud<PointCloud3D::PointXYZPC> exportPointCloud = GPython3D::instance().getExportPointCloud();
    if(path.empty()) {
        path = Util::getExportPath("pointclouds");
    }
    std::string fileName = "point_cloud_" + std::to_string(int(ros::Time::now().toSec())) + ".ply";
    std::string fullPath = path + "/" + fileName;
    pcl::io::savePLYFile(fullPath, exportPointCloud);
    STREAM_DEBUG("ModelExport export3DMap() finished");
}

std::vector<std::pair<grid_map::GridMap, std::shared_ptr<nav_msgs::OccupancyGrid>>> ModelExporter::getMaps() {
    STREAM_DEBUG("ModelExport getGridMaps()");
    return {GPython2D::instance().getExportGridMap()};
}

std::vector<std::shared_ptr<Source>> ModelExporter::getSources() {
    STREAM_DEBUG("ModelExport getSources()");
    std::vector<std::shared_ptr<Source>> sources;
    if (GPython3D::instance().hasEnvironmentCloud()) {
        std::vector<std::shared_ptr<Source>> sources_copy(GPython3D::instance().getSources());
        sources.insert(std::end(sources), std::begin(sources_copy), std::end(sources_copy));
    } else {
        std::vector<std::shared_ptr<SourceInteractive>> sources_copy(GPython2D::instance().getSources());
        sources.insert(std::end(sources), std::begin(sources_copy), std::end(sources_copy));
    }
    return sources;
}

std::vector<Eigen::Vector2f> ModelExporter::getTrajectory() {
    STREAM_DEBUG("ModelExport getTrajectory()");
    std::vector<Sample> samples = SampleManager::instance().getSamples();
    std::vector<Eigen::Vector2f> trajectory;
    for (Sample sample: samples) {
        Eigen::Vector2d pos = sample.get2DPos();
        Eigen::Vector2f posf((float) pos.x(), (float) pos.y());
        trajectory.push_back(posf);
    }
    return trajectory;
}

nav_msgs::OccupancyGrid ModelExporter::setUpCombinedGrid(nav_msgs::OccupancyGrid &slamGrid) {
    STREAM_DEBUG("ModelExport setUpCombinedGrid()");
    nav_msgs::OccupancyGrid combinedGrid;

    STREAM_DEBUG("Set geotiff metadata!");
    combinedGrid.info.resolution = slamGrid.info.resolution;
    combinedGrid.info.width = slamGrid.info.width;
    combinedGrid.info.height = slamGrid.info.height;
    combinedGrid.info.origin.position.x = slamGrid.info.origin.position.x;
    combinedGrid.info.origin.position.y = slamGrid.info.origin.position.y;
    combinedGrid.info.origin.position.z = 0;
    combinedGrid.header.frame_id = "world";
    int gridWidth = combinedGrid.info.width;
    int gridHeight = combinedGrid.info.height;

    STREAM_DEBUG("Set SLAM data!");
    int max_i = gridWidth * gridHeight;
    for (int i = 0; i < max_i; i++) {
        combinedGrid.data.push_back(0.0);

        int y = i / gridWidth;
        int x = i % gridWidth;
        int i_combined = x + y * gridWidth;

        if (slamGrid.data[i] > 0) {
            combinedGrid.data[i_combined] = 100;
        } else if (slamGrid.data[i] == -1) {
            combinedGrid.data[i_combined] = -1;
        }
    }
    return combinedGrid;
}

void ModelExporter::createGeoTiff(const std::string &fileName, const std::string &path, Vector &data,
                                  std::vector<std::shared_ptr<Source>> &sources,
                                  std::vector<Eigen::Vector2f> &trajectory,
                                  nav_msgs::OccupancyGrid &slamGrid,
                                  nav_msgs::OccupancyGrid &combinedGrid,
                                  bool useLogScale) {
    Vector data_copy(data);
    if (useLogScale) {
        data_copy = data.array().log1p();
    }

    geotiff_writer_->setMapFileName(fileName);
    geotiff_writer_->setMapFilePath(path);
    geotiff_writer_->setUnit(Parameters::instance().radiationUnit);
    geotiff_writer_->setupTransforms(combinedGrid);
    geotiff_writer_->setupImageSize();
    geotiff_writer_->setupData(data_copy, turbo_colormap);
    geotiff_writer_->drawBackgroundCheckerboard();
    geotiff_writer_->drawMap(combinedGrid);

    // Draw Trajectory
    Eigen::Vector3f start(trajectory[0].x(), trajectory[0].y(), 0.0);
    geotiff_writer_->drawPath(start, trajectory, 255, 255, 255);

    // Draw all sources
    int i = 0;
    for (std::shared_ptr<Source> &source: sources) {
        if (!source->isConfirmed())
            continue;

        double actualValue = 230;
        double strength1m = computeSourceError2D(i, source->getPos(), combinedGrid, 1, actualValue, 100, false, true);

        Eigen::Vector2f coords2D(source->getPos().x(), source->getPos().y());
        std::string strength = std::to_string(strength1m);
        std::string text = strength.substr(0, strength.find(',') + 3);
        hector_geotiff::MapWriterInterface::Color color(255, 255, 255);
        geotiff_writer_->drawObjectOfInterest(coords2D, text, color, hector_geotiff::Shape::SHAPE_CIRCLE, i);


        i++;
    }
    geotiff_writer_->drawCoords();
    geotiff_writer_->drawScale(data, useLogScale);
    geotiff_writer_->writeGeotiffImage(true);
}

double ModelExporter::computeSourceError2D(int sourceId, const Eigen::Vector3d &sourcePos, nav_msgs::OccupancyGrid &slamGrid,
                                    double radius, double actualValue, int numSamples, bool wallsOn, bool useGlobal) {
    double dAngle = 2 * M_PI / numSamples;
    Matrix evalPositions;
    for (int i = 0; i < numSamples; i++) {
        double angle = i * dAngle;
        Eigen::Vector2d dir(cos(angle), sin(angle));
        Eigen::Vector3d pos = sourcePos;
        pos.x() += (dir * radius).x();
        pos.y() += (dir * radius).y();

        if (!wallsOn) {
            int x = (int) ((pos.x() - slamGrid.info.origin.position.x) / slamGrid.info.resolution);
            int y = (int) ((pos.y() - slamGrid.info.origin.position.y) / slamGrid.info.resolution);
            if (slamGrid.data.at(x + y * slamGrid.info.width) < 0) {
                continue;
            }
        }
        // Add new position to evalPositions in the most ugly way imaginable
        if (useGlobal) {
            evalPositions.conservativeResize(evalPositions.rows() + 1, 2);
            evalPositions.row(evalPositions.rows() - 1) << pos.x(), pos.y();
        } else {
            evalPositions.conservativeResize(evalPositions.rows() + 1, 3);
            evalPositions.row(evalPositions.rows() - 1) << pos.x(), pos.y(), pos.z();
        }
    }
    Vector predictions = GPython::instance().evaluate(evalPositions).mean;
    double mean = predictions.mean();

    // calculate variance
    double var = 0;
    for (int i = 0; i < predictions.rows(); i++) {
        var += pow(predictions(i) - mean, 2);
    }
    var /= predictions.rows();
    double stdVar = sqrt(var);

    // calculate deviation in percent
    double deviation = ((mean - actualValue) * 100) / actualValue;

    STREAM_DEBUG("Source #" << sourceId << " useGlobal " << useGlobal << " WallsOn: " << wallsOn << " Mean: " << mean
                               << " std var: " << stdVar << " deviation: " << deviation);
    return mean;
}