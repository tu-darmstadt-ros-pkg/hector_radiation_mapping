// A custom Geotiff Writer implementation by Jonas Suess and Martin Volz
// Original version by Stefan Kohlbrecher

#ifndef RADIATION_MAPPING_GEOTIFFWRITER_H__
#define RADIATION_MAPPING_GEOTIFFWRITER_H__

#include "hector_geotiff/map_writer_interface.h"

#include <Eigen/Geometry>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <QImage>
#include <QApplication>
#include <QFont>

#include "hector_map_tools/HectorMapTools.h"

#include "util/color.h"

#if  __cplusplus < 201703L
	#include <experimental/filesystem>
	namespace fs = std::experimental::filesystem;
#else
	#include <filesystem>
	namespace fs = std::filesystem;
#endif


namespace geotiff{


class GeotiffWriter : public hector_geotiff::MapWriterInterface
{
    public:
    explicit GeotiffWriter(bool useCheckerboardCacheIn = false);
    virtual ~GeotiffWriter();

    //setUsePrecalcGrid(bool usePrecalc, const Eigen::Vector2f& size);

    void setMapFileName(const std::string& mapFileName);
    void setUnit(const std::string& unit);
    void setMapFilePath(const std::string& mapFilePath);
    void setUseUtcTimeSuffix(bool useSuffix);

    void setupImageSize();
    bool setupTransforms(const nav_msgs::OccupancyGrid& map);
    void setupData(const Vector& values, const Eigen::MatrixX3d& colorMap);
    void drawBackgroundCheckerboard();
    void drawMap(const nav_msgs::OccupancyGrid& map, bool draw_explored_space_grid = true);
    void drawObjectOfInterest(const Eigen::Vector2f& coords, const std::string& txt, const Color& color, const hector_geotiff::Shape& shape);
    void drawObjectOfInterest(const Eigen::Vector2f& coords, const std::string& txt, const Color& color, const hector_geotiff::Shape& shape, int i);
    inline virtual void drawPath(const Eigen::Vector3f& start, const std::vector<Eigen::Vector2f>& points){
      drawPath(start, points, 120,0,240);
    }
    void drawPath(const Eigen::Vector3f& start, const std::vector<Eigen::Vector2f>& points, int color_r, int color_g, int color_b);
    void drawScale(Vector &data, bool logScale);
    void drawCoords();
    std::string getBasePathAndFileName() const;
    void writeGeotiffImage(bool completed);

    /**
     * Applies a color map to all values and returns a Matrix of RGBA colors, one for each value.
     * The colorMap needs to have at least 2 rows representing two RBG colors.
     *
     * @param colorMap Eigen::Matrix<double, Eigen::Dynamic, 3>, A matrix of color samples of a color map, where every
     * row is a RGB color, starting with the color for the smallest value in row 0 and ending with the color for the largest value.
     * @param values Eigen::Matrix<double, Eigen::Dynamic, 1>, A vector of all values, that you want to apply the colormap to.
     * @param invert bool, false by default. Set true if you want to invert the color map.
     * @return result  Eigen::Matrix<double, Eigen::Dynamic, 4>, Each row of this matrix is a RGBA color representing the value
     * in "values" at the same index.
     */
    static Eigen::MatrixX4d applyColorMap(const Eigen::MatrixX3d &colorMap, const Vector &values, bool invert = false, bool logspace = false);
    static Eigen::Vector4d interpolateColorMap(const Eigen::MatrixX3d &colorMap, double x);
    static std::string doubleToStr(double value, int digits);

    protected:

    void transformPainterToImgCoords(QPainter& painter);
    void drawCross(QPainter& painter, const Eigen::Vector2f& coords);
    void drawArrow(QPainter& painter);
    void drawCoordSystem(QPainter& painter);

    float resolution = std::numeric_limits<float>::quiet_NaN();
    Eigen::Vector2f origin;

    int resolutionFactor = 3;
    float resolutionFactorf = std::numeric_limits<float>::quiet_NaN();

    bool useCheckerboardCache;
    bool use_utc_time_suffix_;

    float pixelsPerMapMeter = std::numeric_limits<float>::quiet_NaN();
    float pixelsPerGeoTiffMeter = std::numeric_limits<float>::quiet_NaN();

    Eigen::Vector2i minCoordsMap;
    Eigen::Vector2i maxCoordsMap;

    Eigen::Vector2i sizeMap;
    Eigen::Vector2f sizeMapf;

    Eigen::Vector2f rightBottomMarginMeters;
    Eigen::Vector2f rightBottomMarginPixelsf;
    Eigen::Vector2i rightBottomMarginPixels;

    Eigen::Vector2f leftTopMarginMeters;

    Eigen::Vector2f totalMeters;

    Eigen::Vector2i geoTiffSizePixels;

    Eigen::Vector2f mapOrigInGeotiff;
    Eigen::Vector2f mapEndInGeotiff;

    std::string map_file_name_;
    std::string map_file_path_;

    QImage image;
    QImage checkerboard_cache;
    QApplication* app;
    QString font_family_;
    QFont map_draw_font_;

    HectorMapTools::CoordinateTransformer<float> world_map_transformer_;
    HectorMapTools::CoordinateTransformer<float> map_geo_transformer_;
    HectorMapTools::CoordinateTransformer<float> world_geo_transformer_;

    nav_msgs::MapMetaData cached_map_meta_data_;

    Eigen::MatrixX4d colorMapMatrix_;
    Eigen::MatrixX3d colorMap_;
    double maxValue_, minValue_;
    std::string unit_;
};

}

#endif
