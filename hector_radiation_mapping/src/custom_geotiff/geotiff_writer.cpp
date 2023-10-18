// A custom Geotiff Writer implementation by Jonas Suess and Martin Volz
// Original version by Stefan Kohlbrecher

#include <ros/console.h>

#include <QFile>
#include <QImageWriter>
#include <QPainter>
//#include <QtCore/QDateTime>
#include <QTime>
#include <QTextStream>
#include <QFontDatabase>

#include <ros/package.h>

#if  __cplusplus < 201703L
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else

#include <filesystem>
#include "util/print.h"

namespace fs = std::filesystem;
#endif

#include "pch.h"
#include "custom_geotiff/geotiff_writer.h"

namespace geotiff {

    namespace {
        Eigen::MatrixX3d turbo_colormap = (Eigen::Matrix<double, 256, 3>()
                << 0.18995, 0.07176, 0.23217, 0.19483, 0.08339, 0.26149, 0.19956, 0.09498, 0.29024, 0.20415, 0.10652, 0.31844, 0.20860, 0.11802, 0.34607, 0.21291, 0.12947, 0.37314, 0.21708, 0.14087, 0.39964, 0.22111, 0.15223, 0.42558, 0.22500, 0.16354, 0.45096, 0.22875, 0.17481, 0.47578, 0.23236, 0.18603, 0.50004, 0.23582, 0.19720, 0.52373, 0.23915, 0.20833, 0.54686, 0.24234, 0.21941, 0.56942, 0.24539, 0.23044, 0.59142, 0.24830, 0.24143, 0.61286, 0.25107, 0.25237, 0.63374, 0.25369, 0.26327, 0.65406, 0.25618, 0.27412, 0.67381, 0.25853, 0.28492, 0.69300, 0.26074, 0.29568, 0.71162, 0.26280, 0.30639, 0.72968, 0.26473, 0.31706, 0.74718, 0.26652, 0.32768, 0.76412, 0.26816, 0.33825, 0.78050, 0.26967, 0.34878, 0.79631, 0.27103, 0.35926, 0.81156, 0.27226, 0.36970, 0.82624, 0.27334, 0.38008, 0.84037, 0.27429, 0.39043, 0.85393, 0.27509, 0.40072, 0.86692, 0.27576, 0.41097, 0.87936, 0.27628, 0.42118, 0.89123, 0.27667, 0.43134, 0.90254, 0.27691, 0.44145, 0.91328, 0.27701, 0.45152, 0.92347, 0.27698, 0.46153, 0.93309, 0.27680, 0.47151, 0.94214, 0.27648, 0.48144, 0.95064, 0.27603, 0.49132, 0.95857, 0.27543, 0.50115, 0.96594, 0.27469, 0.51094, 0.97275, 0.27381, 0.52069, 0.97899, 0.27273, 0.53040, 0.98461, 0.27106, 0.54015, 0.98930, 0.26878, 0.54995, 0.99303, 0.26592, 0.55979, 0.99583, 0.26252, 0.56967, 0.99773, 0.25862, 0.57958, 0.99876, 0.25425, 0.58950, 0.99896, 0.24946, 0.59943, 0.99835, 0.24427, 0.60937, 0.99697, 0.23874, 0.61931, 0.99485, 0.23288, 0.62923, 0.99202, 0.22676, 0.63913, 0.98851, 0.22039, 0.64901, 0.98436, 0.21382, 0.65886, 0.97959, 0.20708, 0.66866, 0.97423, 0.20021, 0.67842, 0.96833, 0.19326, 0.68812, 0.96190, 0.18625, 0.69775, 0.95498, 0.17923, 0.70732, 0.94761, 0.17223, 0.71680, 0.93981, 0.16529, 0.72620, 0.93161, 0.15844, 0.73551, 0.92305, 0.15173, 0.74472, 0.91416, 0.14519, 0.75381, 0.90496, 0.13886, 0.76279, 0.89550, 0.13278, 0.77165, 0.88580, 0.12698, 0.78037, 0.87590, 0.12151, 0.78896, 0.86581, 0.11639, 0.79740, 0.85559, 0.11167, 0.80569, 0.84525, 0.10738, 0.81381, 0.83484, 0.10357, 0.82177, 0.82437, 0.10026, 0.82955, 0.81389, 0.09750, 0.83714, 0.80342, 0.09532, 0.84455, 0.79299, 0.09377, 0.85175, 0.78264, 0.09287, 0.85875, 0.77240, 0.09267, 0.86554, 0.76230, 0.09320, 0.87211, 0.75237, 0.09451, 0.87844, 0.74265, 0.09662, 0.88454, 0.73316, 0.09958, 0.89040, 0.72393, 0.10342, 0.89600, 0.71500, 0.10815, 0.90142, 0.70599, 0.11374, 0.90673, 0.69651, 0.12014, 0.91193, 0.68660, 0.12733, 0.91701, 0.67627, 0.13526, 0.92197, 0.66556, 0.14391, 0.92680, 0.65448, 0.15323, 0.93151, 0.64308, 0.16319, 0.93609, 0.63137, 0.17377, 0.94053, 0.61938, 0.18491, 0.94484, 0.60713, 0.19659, 0.94901, 0.59466, 0.20877, 0.95304, 0.58199, 0.22142, 0.95692, 0.56914, 0.23449, 0.96065, 0.55614, 0.24797, 0.96423, 0.54303, 0.26180, 0.96765, 0.52981, 0.27597, 0.97092, 0.51653, 0.29042, 0.97403, 0.50321, 0.30513, 0.97697, 0.48987, 0.32006, 0.97974, 0.47654, 0.33517, 0.98234, 0.46325, 0.35043, 0.98477, 0.45002, 0.36581, 0.98702, 0.43688, 0.38127, 0.98909, 0.42386, 0.39678, 0.99098, 0.41098, 0.41229, 0.99268, 0.39826, 0.42778, 0.99419, 0.38575, 0.44321, 0.99551, 0.37345, 0.45854, 0.99663, 0.36140, 0.47375, 0.99755, 0.34963, 0.48879, 0.99828, 0.33816, 0.50362, 0.99879, 0.32701, 0.51822, 0.99910, 0.31622, 0.53255, 0.99919, 0.30581, 0.54658, 0.99907, 0.29581, 0.56026, 0.99873, 0.28623, 0.57357, 0.99817, 0.27712, 0.58646, 0.99739, 0.26849, 0.59891, 0.99638, 0.26038, 0.61088, 0.99514, 0.25280, 0.62233, 0.99366, 0.24579, 0.63323, 0.99195, 0.23937, 0.64362, 0.98999, 0.23356, 0.65394, 0.98775, 0.22835, 0.66428, 0.98524, 0.22370, 0.67462, 0.98246, 0.21960, 0.68494, 0.97941, 0.21602, 0.69525, 0.97610, 0.21294, 0.70553, 0.97255, 0.21032, 0.71577, 0.96875, 0.20815, 0.72596, 0.96470, 0.20640, 0.73610, 0.96043, 0.20504, 0.74617, 0.95593, 0.20406, 0.75617, 0.95121, 0.20343, 0.76608, 0.94627, 0.20311, 0.77591, 0.94113, 0.20310, 0.78563, 0.93579, 0.20336, 0.79524, 0.93025, 0.20386, 0.80473, 0.92452, 0.20459, 0.81410, 0.91861, 0.20552, 0.82333, 0.91253, 0.20663, 0.83241, 0.90627, 0.20788, 0.84133, 0.89986, 0.20926, 0.85010, 0.89328, 0.21074, 0.85868, 0.88655, 0.21230, 0.86709, 0.87968, 0.21391, 0.87530, 0.87267, 0.21555, 0.88331, 0.86553, 0.21719, 0.89112, 0.85826, 0.21880, 0.89870, 0.85087, 0.22038, 0.90605, 0.84337, 0.22188, 0.91317, 0.83576, 0.22328, 0.92004, 0.82806, 0.22456, 0.92666, 0.82025, 0.22570, 0.93301, 0.81236, 0.22667, 0.93909, 0.80439, 0.22744, 0.94489, 0.79634, 0.22800, 0.95039, 0.78823, 0.22831, 0.95560, 0.78005, 0.22836, 0.96049, 0.77181, 0.22811, 0.96507, 0.76352, 0.22754, 0.96931, 0.75519, 0.22663, 0.97323, 0.74682, 0.22536, 0.97679, 0.73842, 0.22369, 0.98000, 0.73000, 0.22161, 0.98289, 0.72140, 0.21918, 0.98549, 0.71250, 0.21650, 0.98781, 0.70330, 0.21358, 0.98986, 0.69382, 0.21043, 0.99163, 0.68408, 0.20706, 0.99314, 0.67408, 0.20348, 0.99438, 0.66386, 0.19971, 0.99535, 0.65341, 0.19577, 0.99607, 0.64277, 0.19165, 0.99654, 0.63193, 0.18738, 0.99675, 0.62093, 0.18297, 0.99672, 0.60977, 0.17842, 0.99644, 0.59846, 0.17376, 0.99593, 0.58703, 0.16899, 0.99517, 0.57549, 0.16412, 0.99419, 0.56386, 0.15918, 0.99297, 0.55214, 0.15417, 0.99153, 0.54036, 0.14910, 0.98987, 0.52854, 0.14398, 0.98799, 0.51667, 0.13883, 0.98590, 0.50479, 0.13367, 0.98360, 0.49291, 0.12849, 0.98108, 0.48104, 0.12332, 0.97837, 0.46920, 0.11817, 0.97545, 0.45740, 0.11305, 0.97234, 0.44565, 0.10797, 0.96904, 0.43399, 0.10294, 0.96555, 0.42241, 0.09798, 0.96187, 0.41093, 0.09310, 0.95801, 0.39958, 0.08831, 0.95398, 0.38836, 0.08362, 0.94977, 0.37729, 0.07905, 0.94538, 0.36638, 0.07461, 0.94084, 0.35566, 0.07031, 0.93612, 0.34513, 0.06616, 0.93125, 0.33482, 0.06218, 0.92623, 0.32473, 0.05837, 0.92105, 0.31489, 0.05475, 0.91572, 0.30530, 0.05134, 0.91024, 0.29599, 0.04814, 0.90463, 0.28696, 0.04516, 0.89888, 0.27824, 0.04243, 0.89298, 0.26981, 0.03993, 0.88691, 0.26152, 0.03753, 0.88066, 0.25334, 0.03521, 0.87422, 0.24526, 0.03297, 0.86760, 0.23730, 0.03082, 0.86079, 0.22945, 0.02875, 0.85380, 0.22170, 0.02677, 0.84662, 0.21407, 0.02487, 0.83926, 0.20654, 0.02305, 0.83172, 0.19912, 0.02131, 0.82399, 0.19182, 0.01966, 0.81608, 0.18462, 0.01809, 0.80799, 0.17753, 0.01660, 0.79971, 0.17055, 0.01520, 0.79125, 0.16368, 0.01387, 0.78260, 0.15693, 0.01264, 0.77377, 0.15028, 0.01148, 0.76476, 0.14374, 0.01041, 0.75556, 0.13731, 0.00942, 0.74617, 0.13098, 0.00851, 0.73661, 0.12477, 0.00769, 0.72686, 0.11867, 0.00695, 0.71692, 0.11268, 0.00629, 0.70680, 0.10680, 0.00571, 0.69650, 0.10102, 0.00522, 0.68602, 0.09536, 0.00481, 0.67535, 0.08980, 0.00449, 0.66449, 0.08436, 0.00424, 0.65345, 0.07902, 0.00408, 0.64223, 0.07380, 0.00401, 0.63082, 0.06868, 0.00401, 0.61923, 0.06367, 0.00410, 0.60746, 0.05878, 0.00427, 0.59550, 0.05399, 0.00453, 0.58336, 0.04931, 0.00486, 0.57103, 0.04474, 0.00529, 0.55852, 0.04028, 0.00579, 0.54583, 0.03593, 0.00638, 0.53295, 0.03169, 0.00705, 0.51989, 0.02756, 0.00780, 0.50664, 0.02354, 0.00863, 0.49321, 0.01963, 0.00955, 0.47960, 0.01583, 0.01055).finished();
    }

    GeotiffWriter::GeotiffWriter(bool useCheckerboardCacheIn)
            : useCheckerboardCache(useCheckerboardCacheIn), use_utc_time_suffix_(true) {
        cached_map_meta_data_.height = -1;
        cached_map_meta_data_.width = -1;
        cached_map_meta_data_.resolution = -1.0f;

        int fake_argc = 3;
        char *fake_argv[3] = {new char[15], new char[10], new char[10]};
        strcpy(fake_argv[0], "geotiff_writer");
        strcpy(fake_argv[1], "-platform");
        strcpy(fake_argv[2], "offscreen"); // Set the env QT_DEBUG_PLUGINS to 1 to see available platforms

        STREAM_DEBUG("Creating application with offscreen platform.");
        //Create a QApplication cause otherwise drawing text will crash
        app = new QApplication(fake_argc, fake_argv);
        delete[] fake_argv[0];
        delete[] fake_argv[1];
        delete[] fake_argv[2];
        STREAM_DEBUG("Created application");

        std::string font_path = ros::package::getPath("hector_geotiff") + "/fonts/Roboto-Regular.ttf";
        int id = QFontDatabase::addApplicationFont(QString::fromStdString(font_path));
        font_family_ = QFontDatabase::applicationFontFamilies(id).at(0);

        map_file_name_ = "";
        map_file_path_ = "";
    }

    GeotiffWriter::~GeotiffWriter() {
        delete app;
    }

    void GeotiffWriter::setupData(const Vector &values, const Eigen::MatrixX3d &colorMap) {
        this->colorMap_ = colorMap;
        this->colorMapMatrix_ = applyColorMap(colorMap, values, false, false);
        maxValue_ = values.maxCoeff();
        minValue_ = values.minCoeff();
    }

    void GeotiffWriter::drawScale(Vector &data, bool logScale) {
        QPainter qPainter(&image);
        QFont font(font_family_);
        font.setPixelSize(9 * resolutionFactor);
        qPainter.setFont(font);

        int numBars = pixelsPerGeoTiffMeter * 8;
        int offX = pixelsPerGeoTiffMeter / 2;
        int offY = 4 * pixelsPerGeoTiffMeter;
        int barWidth = pixelsPerGeoTiffMeter / 2;

        double maxValue = data.maxCoeff();
        double minValue = data.minCoeff();

        Vector steps = Eigen::VectorXd::Zero(numBars);
        for (int i = 0; i < numBars; i++) {
            double t;
            if (logScale) {
                t = log1p(minValue + (maxValue - minValue) * (double(i) / numBars));
            } else {
                t = minValue + (maxValue - minValue) * (double(i) / numBars);
            }
            steps[i] = t;
        }
        Eigen::MatrixX4d colors = applyColorMap(turbo_colormap, steps);
        int r, g, b;
        for (int i = 0; i < numBars; i++) {
            r = int(colors.row(i).x() * 255);
            g = int(colors.row(i).y() * 255);
            b = int(colors.row(i).z() * 255);
            qPainter.fillRect(offX, offY + numBars - i, barWidth, 1, QColor(r, g, b));
        }

        qPainter.setPen(QColor(0, 0, 0));
        qPainter.drawRect(offX, offY, barWidth, (numBars + 1));
        qPainter.drawLine(offX + barWidth, offY, offX + 1.5 * barWidth, offY);
        qPainter.drawLine(offX + barWidth, offY + 0.25 * (numBars + 1), offX + 1.5 * barWidth,
                          offY + 0.25 * (numBars + 1));
        qPainter.drawLine(offX + barWidth, offY + 0.5 * (numBars + 1), offX + 1.5 * barWidth,
                          offY + 0.5 * (numBars + 1));
        qPainter.drawLine(offX + barWidth, offY + 0.75 * (numBars + 1), offX + 1.5 * barWidth,
                          offY + 0.75 * (numBars + 1));
        qPainter.drawLine(offX + barWidth, offY + (numBars + 1), offX + 1.5 * barWidth, offY + (numBars + 1));

        double v50 = minValue + (maxValue - minValue) * 0.50;
        qPainter.setPen(QColor(0, 50, 140));
        qPainter.drawText(offX + 2 * barWidth, offY + 20, QString(doubleToStr(maxValue, 2).c_str()));
        qPainter.drawText(offX + 2 * barWidth, offY + 10 + 0.5 * (numBars + 1), QString(doubleToStr(v50, 2).c_str()));
        qPainter.drawText(offX + 2 * barWidth, offY + (numBars + 1), QString(doubleToStr(minValue, 2).c_str()));

        if (logScale) {
            qPainter.drawText(offX, offY - 30, QString("Scalling: logp1(x)"));
        } else {
            qPainter.drawText(offX, offY - 30, QString("Scaling: linear"));
        }

        // Unit
        qPainter.drawText(offX, offY + (numBars + 1) + 45, QString(("Unit: " + unit_).c_str()));
        qPainter.drawText(offX, offY + (numBars + 1) + 23 + pixelsPerGeoTiffMeter, QString("Sources:"));
        qPainter.drawText(offX, offY + (numBars + 1) + 50 + pixelsPerGeoTiffMeter, QString("(at 1m)"));
        qPainter.setFont(map_draw_font_);
        //map_draw_font_.setPixelSize(6 * resolutionFactor);
    }

    void GeotiffWriter::drawMap(const nav_msgs::OccupancyGrid &map, bool draw_explored_space_grid) {
        QPainter qPainter(&image);
        transformPainterToImgCoords(qPainter);
        QRectF map_cell_grid_tile(0.0f, 0.0f, resolutionFactor, resolutionFactor);
        QBrush occupied_brush(QColor(0, 0, 0));
        int width = map.info.width;
        float explored_space_grid_resolution_pixels = pixelsPerGeoTiffMeter * 0.5f;
        float yGeo = 0.0f;
        float currYLimit = 0.0f;
        bool drawY = false;

        for (int y = minCoordsMap[1]; y < maxCoordsMap[1]; ++y) {
            float xGeo = 0.0f;
            if (yGeo >= currYLimit) {
                drawY = true;
            }

            float currXLimit = 0.0f;
            bool drawX = false;

            for (int x = minCoordsMap[0]; x < maxCoordsMap[0]; ++x) {
                unsigned int i = y * width + x;
                int8_t data = map.data[i];

                if (xGeo >= currXLimit) {
                    drawX = true;
                }
                if (data > 0) {
                    qPainter.fillRect(mapOrigInGeotiff.x() + xGeo, mapOrigInGeotiff.y() + yGeo, resolutionFactorf,
                                      resolutionFactorf, occupied_brush);
                } else if (data == 0) {
                    if (i < colorMapMatrix_.rows()) {
                        auto color = colorMapMatrix_.row(i);
                        QBrush brush(QColor(int(255.0 * color[0]), int(255.0 * color[1]), int(255.0 * color[2])));
                        qPainter.fillRect(mapOrigInGeotiff.x() + xGeo, mapOrigInGeotiff.y() + yGeo, resolutionFactorf,
                                          resolutionFactorf, brush);
                    }
                }
                if (drawX) {
                    currXLimit += explored_space_grid_resolution_pixels;
                    drawX = false;
                }
                xGeo += resolutionFactorf;
            }
            if (drawY) {
                drawY = false;
                currYLimit += explored_space_grid_resolution_pixels;
            }
            yGeo += resolutionFactorf;
        }
    }

    void GeotiffWriter::setMapFileName(const std::string &mapFileName) {
        map_file_name_ = mapFileName;

        if (use_utc_time_suffix_) {
            QTime now(QTime::currentTime());
            std::string current_time_string = now.toString(Qt::ISODate).toStdString();

            map_file_name_ += "_" + current_time_string;
        }
    }

    void GeotiffWriter::setMapFilePath(const std::string &mapFilePath) {
        map_file_path_ = mapFilePath;
    }

    void GeotiffWriter::setUseUtcTimeSuffix(bool useSuffix) {
        use_utc_time_suffix_ = useSuffix;
    }

    bool GeotiffWriter::setupTransforms(const nav_msgs::OccupancyGrid &map) {
        resolution = static_cast<float>(map.info.resolution);
        origin = Eigen::Vector2f(map.info.origin.position.x, map.info.origin.position.y);

        resolutionFactor = 3;
        resolutionFactorf = static_cast<float>(resolutionFactor);

        pixelsPerMapMeter = 1.0f / map.info.resolution;
        pixelsPerGeoTiffMeter = pixelsPerMapMeter * static_cast<float>(resolutionFactor);

        minCoordsMap = Eigen::Vector2i::Zero();
        maxCoordsMap = Eigen::Vector2i(map.info.width, map.info.height);

        if (!HectorMapTools::getMapExtends(map, minCoordsMap, maxCoordsMap)) {
            STREAM_DEBUG("Cannot determine map extends!");
            return false;
        }

        sizeMap = Eigen::Vector2i(maxCoordsMap - minCoordsMap);
        sizeMapf = ((maxCoordsMap - minCoordsMap).cast<float>());


        rightBottomMarginMeters = Eigen::Vector2f(1.0f, 1.0f);
        rightBottomMarginPixelsf = Eigen::Vector2f(rightBottomMarginMeters.array() * pixelsPerGeoTiffMeter);
        rightBottomMarginPixels = ((rightBottomMarginPixelsf.array() + 0.5f).cast<int>());

        leftTopMarginMeters = Eigen::Vector2f(3.0f, 3.0f);

        totalMeters = (rightBottomMarginMeters + sizeMapf * map.info.resolution + leftTopMarginMeters);
        //std::cout << "\n" << totalMeters;

        totalMeters.x() = ceil(totalMeters.x());
        totalMeters.y() = ceil(totalMeters.y());
        //std::cout << "\n" << totalMeters;

        geoTiffSizePixels = ((totalMeters.array() * pixelsPerGeoTiffMeter).cast<int>());


        mapOrigInGeotiff = (rightBottomMarginPixelsf);
        mapEndInGeotiff = (rightBottomMarginPixelsf + sizeMapf * resolutionFactorf);
        //std::cout << "\n mapOrig\n" << mapOrigInGeotiff;
        //std::cout << "\n mapOrig\n" << mapEndInGeotiff;

        world_map_transformer_.setTransforms(map);

        map_geo_transformer_.setTransformsBetweenCoordSystems(mapOrigInGeotiff, mapEndInGeotiff,
                                                              minCoordsMap.cast<float>(),
                                                              maxCoordsMap.cast<float>());

        /*
        Eigen::Vector2f temp_zero_map_g (map_geo_transformer_.getC2Coords(Eigen::Vector2f::Zero()));

        Eigen::Vector2f temp_zero_map_g_floor (floor(temp_zero_map_g.x()), floor(temp_zero_map_g.x()));

        Eigen::Vector2f diff (temp_zero_map_g - temp_zero_map_g_floor);

        map*/


        Eigen::Vector2f p1_w(Eigen::Vector2f::Zero());
        Eigen::Vector2f p2_w(Eigen::Vector2f(100.0f, 100.0f));

        Eigen::Vector2f p1_m(world_map_transformer_.getC2Coords(p1_w));
        Eigen::Vector2f p2_m(world_map_transformer_.getC2Coords(p2_w));

        Eigen::Vector2f p1_g(map_geo_transformer_.getC2Coords(p1_m));
        Eigen::Vector2f p2_g(map_geo_transformer_.getC2Coords(p2_m));

        world_geo_transformer_.setTransformsBetweenCoordSystems(p1_g, p2_g, p1_w, p2_w);

        map_draw_font_ = QFont(font_family_);
        map_draw_font_.setPixelSize(6 * resolutionFactor);

        if (useCheckerboardCache) {

            if ((cached_map_meta_data_.height != map.info.height) ||
                (cached_map_meta_data_.width != map.info.width) ||
                (cached_map_meta_data_.resolution = map.info.resolution)) {

                cached_map_meta_data_ = map.info;

                Eigen::Vector2f img_size(Eigen::Vector2f(map.info.width, map.info.height) * resolutionFactorf +
                                         (rightBottomMarginMeters + leftTopMarginMeters) * pixelsPerGeoTiffMeter);
                checkerboard_cache = QImage(img_size.y(), img_size.x(), QImage::Format_RGB32);

                QPainter qPainter(&image);
                transformPainterToImgCoords(qPainter);

                QBrush c1 = QBrush(QColor(226, 226, 227));
                QBrush c2 = QBrush(QColor(237, 237, 238));
                QRectF background_grid_tile(0.0f, 0.0f, pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter);


                int xMaxGeo = geoTiffSizePixels[0];
                int yMaxGeo = geoTiffSizePixels[1];

                for (int y = 0; y < yMaxGeo; ++y) {
                    for (int x = 0; x < xMaxGeo; ++x) {
                        //std::cout << "\n" << x << " " << y;

                        if ((x + y) % 2 == 0) {
                            //qPainter.fillRect(background_grid_tile, c1);
                            qPainter.fillRect(static_cast<float>(x) * pixelsPerGeoTiffMeter,
                                              static_cast<float>(y) * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter,
                                              pixelsPerGeoTiffMeter, c1);
                        } else {
                            //qPainter.fillRect(background_grid_tile, c2);
                            qPainter.fillRect(static_cast<float>(x) * pixelsPerGeoTiffMeter,
                                              static_cast<float>(y) * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter,
                                              pixelsPerGeoTiffMeter, c2);
                        }
                        //background_grid_tile.moveTo(QPointF(static_cast<float>(x)*pixelsPerGeoTiffMeter,static_cast<float>(y)*pixelsPerGeoTiffMeter));
                    }
                }
            }
        }
        return true;
    }

    void GeotiffWriter::setupImageSize() {
        bool painter_rotate = true;
        int xMaxGeo = geoTiffSizePixels[0];
        int yMaxGeo = geoTiffSizePixels[1];

        if (!useCheckerboardCache) {
            if (painter_rotate) {
                image = QImage(yMaxGeo, xMaxGeo, QImage::Format_RGB32);
            } else {
                image = QImage(xMaxGeo, yMaxGeo, QImage::Format_RGB32);
            }
            QPainter qPainter(&image);
            QBrush grey = QBrush(QColor(128, 128, 128));
            qPainter.fillRect(image.rect(), grey);
        }
    }

    void GeotiffWriter::drawBackgroundCheckerboard() {
        int xMaxGeo = geoTiffSizePixels[0];
        int yMaxGeo = geoTiffSizePixels[1];
        bool painter_rotate = true;

        if (!useCheckerboardCache) {
            QPainter qPainter(&image);

            if (painter_rotate) {
                transformPainterToImgCoords(qPainter);
            }

            //*********************** Background checkerboard pattern **********************
            QBrush c1 = QBrush(QColor(226, 226, 227));
            QBrush c2 = QBrush(QColor(237, 237, 238));

            for (int y = 0; y < yMaxGeo; y += pixelsPerGeoTiffMeter) {
                for (int x = 0; x < xMaxGeo; x += pixelsPerGeoTiffMeter) {
                    //std::cout << "\n" << x << " " << y;

                    if (((x + y) / (int) pixelsPerGeoTiffMeter) % 2 == 0) {
                        QRect rect(static_cast<float>(x), static_cast<float>(y),
                                   pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter);
                        qPainter.fillRect(rect, c1);
                    } else {
                        QRect rect(static_cast<float>(x), static_cast<float>(y),
                                   pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter);
                        qPainter.fillRect(rect, c2);
                        //qPainter.fillRect(static_cast<float>(x) * pixelsPerGeoTiffMeter,
                        //                  static_cast<float>(y) * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter,
                        //                  pixelsPerGeoTiffMeter, c2);
                    }
                }
            }
        } else {
            image = checkerboard_cache.copy(0, 0, geoTiffSizePixels[0], geoTiffSizePixels[1]);
        }
    }

    void GeotiffWriter::drawObjectOfInterest(const Eigen::Vector2f &coords, const std::string &txt, const Color &color,
                                             const hector_geotiff::Shape &shape) {
        QPainter qPainter(&image);
        transformPainterToImgCoords(qPainter);

        Eigen::Vector2f coords_g(world_geo_transformer_.getC2Coords(coords));
        qPainter.translate(coords_g[0], coords_g[1]);
        qPainter.rotate(90);
        qPainter.setRenderHint(QPainter::Antialiasing, true);

        float radius = pixelsPerGeoTiffMeter * 0.16f;

        QRectF shape_rect(-radius, -radius, radius * 2.0f, radius * 2.0f);
        qPainter.save();

        QBrush tmpBrush(QColor(color.r, color.g, color.b));
        QPen tmpPen(QColor(0, 0, 0));
        qPainter.setBrush(tmpBrush);
        qPainter.setPen(tmpPen);

        if (shape == hector_geotiff::SHAPE_CIRCLE) {
            qPainter.drawEllipse(shape_rect);
        } else if (shape == hector_geotiff::SHAPE_DIAMOND) {
            qPainter.rotate(45);
            qPainter.drawRect(shape_rect);
        }

        qPainter.restore();
        qPainter.setFont(map_draw_font_);

        QRectF text_rect(-radius * 10.f, -radius, radius * 20.f, radius * 2.f);
        qPainter.setPen(Qt::white);
        qPainter.setBrush(Qt::black);
        qPainter.scale(-1.0, 1.0);
        qPainter.translate(0, -2 * radius);
        qPainter.drawText(text_rect, Qt::AlignCenter, QString(txt.c_str()));
    }

    void GeotiffWriter::drawObjectOfInterest(const Eigen::Vector2f &coords, const std::string &txt, const Color &color,
                                             const hector_geotiff::Shape &shape, int i) {
        QPainter qPainter(&image);
        transformPainterToImgCoords(qPainter);

        Eigen::Vector2f coords_g(world_geo_transformer_.getC2Coords(coords));
        qPainter.translate(coords_g[0], coords_g[1]);
        qPainter.rotate(90);
        qPainter.setRenderHint(QPainter::Antialiasing, true);

        float radius = pixelsPerGeoTiffMeter * 0.16f;

        QRectF shape_rect(-radius, -radius, radius * 2.0f, radius * 2.0f);
        qPainter.save();

        QBrush tmpBrush(QColor(color.r, color.g, color.b));
        QPen tmpPen(QColor(0, 0, 0));
        qPainter.setBrush(tmpBrush);
        qPainter.setPen(tmpPen);

        if (shape == hector_geotiff::SHAPE_CIRCLE) {
            qPainter.drawEllipse(shape_rect);
        } else if (shape == hector_geotiff::SHAPE_DIAMOND) {
            qPainter.rotate(45);
            qPainter.drawRect(shape_rect);
        }

        qPainter.restore();
        qPainter.setFont(map_draw_font_);
        qPainter.setBrush(QColor(0, 0, 0));
        qPainter.setPen(QColor(0, 0, 0));
        qPainter.scale(-1.0, 1.0);
        qPainter.drawText(-5, 7, QString(std::to_string(i).c_str()));

        /// Draw Source in Legend
        qPainter.resetTransform();
        qPainter.translate(pixelsPerGeoTiffMeter * 3 / 4, 14.5 * pixelsPerGeoTiffMeter + i * pixelsPerGeoTiffMeter);
        qPainter.rotate(90);
        qPainter.setRenderHint(QPainter::Antialiasing, true);
        qPainter.setBrush(tmpBrush);
        qPainter.setPen(tmpPen);

        radius = pixelsPerGeoTiffMeter / 4;
        QRectF sourceCircle(-radius, -radius, radius * 2.0f, radius * 2.0f);
        if (shape == hector_geotiff::SHAPE_CIRCLE) {
            qPainter.drawEllipse(sourceCircle);
        } else if (shape == hector_geotiff::SHAPE_DIAMOND) {
            qPainter.rotate(45);
            qPainter.drawRect(sourceCircle);
        }

        QFont font(font_family_);
        font.setPixelSize(9 * resolutionFactor);
        qPainter.setFont(font);
        qPainter.setBrush(QColor(0, 50, 140));
        qPainter.setPen(QColor(0, 50, 140));
        qPainter.resetTransform();
        qPainter.drawText(1.5 * pixelsPerGeoTiffMeter, 14.5 * pixelsPerGeoTiffMeter + i * pixelsPerGeoTiffMeter + 10,
                          QString(txt.c_str()));

        qPainter.setBrush(QColor(0, 0, 0));
        qPainter.setPen(QColor(0, 0, 0));
        qPainter.drawText((3 * pixelsPerGeoTiffMeter) / 4 - 8,
                          14.5 * pixelsPerGeoTiffMeter + i * pixelsPerGeoTiffMeter + 10,
                          QString(std::to_string(i).c_str()));
        //qPainter.setFont(map_draw_font_);
    }

    void GeotiffWriter::drawPath(const Eigen::Vector3f &start, const std::vector<Eigen::Vector2f> &points, int color_r,
                                 int color_g, int color_b) {
        QPainter qPainter(&image);
        transformPainterToImgCoords(qPainter);
        Eigen::Vector2f start_geo(world_geo_transformer_.getC2Coords(start.head<2>()));
        size_t size = points.size();

        QPolygonF polygon;
        polygon.reserve(size);
        polygon.push_back(QPointF(start_geo.x(), start_geo.y()));

        for (size_t i = 0; i < size; ++i) {
            const Eigen::Vector2f vec(world_geo_transformer_.getC2Coords(points[i]));
            polygon.push_back(QPointF(vec.x(), vec.y()));
        }

        QPen pen(qPainter.pen());
        pen.setColor(QColor(color_r, color_g, color_b));
        pen.setWidth(3);
        qPainter.setRenderHint(QPainter::Antialiasing, true);
        qPainter.setPen(pen);
        qPainter.drawPolyline(polygon);
        qPainter.save();
        qPainter.translate(start_geo.x(), start_geo.y());
        qPainter.rotate(start.z());

        drawArrow(qPainter);
        //drawCoordSystem(qPainter);
        qPainter.restore();
    }

    std::string GeotiffWriter::getBasePathAndFileName() const {
        return std::string(map_file_path_ + "/" + map_file_name_);
    }

    void GeotiffWriter::writeGeotiffImage(bool completed) {
        //Only works with recent Qt versions
        //QDateTime now (QDateTime::currentDateTimeUtc());
        //std::string current_time_string = now.toString(Qt::ISODate).toStdString();
        std::string complete_file_string;
        if (completed) {
            complete_file_string = map_file_path_ + "/" + map_file_name_;
        } else {
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::stringstream start_ss;
            start_ss << std::put_time(&tm, "%Y-%m-%d");

            complete_file_string = map_file_path_ + "/autosave";
            std::error_code error;
            if (!fs::exists(complete_file_string.c_str())) {
                fs::create_directory(complete_file_string.c_str(), error);
                if (error) {
                    ROS_ERROR("Can't create autosave folder");
                    return;
                }
            }

            complete_file_string += "/" + start_ss.str();
            if (!fs::exists(complete_file_string.c_str())) {
                fs::create_directory(complete_file_string.c_str(), error);
                if (error) {
                    ROS_ERROR("Can't create folder in autosave");
                    return;
                }
            }

            complete_file_string += "/" + map_file_name_;
        }

        QImageWriter imageWriter(QString::fromStdString(complete_file_string + ".tif"));
        imageWriter.setCompression(1);

        bool success = imageWriter.write(image);

        std::string tfw_file_name(complete_file_string + ".tfw");
        QFile tfwFile(QString::fromStdString(tfw_file_name));

        tfwFile.open(QIODevice::WriteOnly);

        QTextStream out(&tfwFile);

        float resolution_geo = resolution / resolutionFactorf;

        QString resolution_string;
        resolution_string.setNum(resolution_geo, 'f', 10);

        //positive x resolution
        out << resolution_string << "\n";

        QString zero_string;
        zero_string.setNum(0.0f, 'f', 10);

        //rotation, translation
        out << zero_string << "\n" << zero_string << "\n";

        //negative y resolution
        out << "-" << resolution_string << "\n";

        QString top_left_string_x;
        QString top_left_string_y;
        Eigen::Vector2f zero_geo_w(world_geo_transformer_.getC1Coords((geoTiffSizePixels.array() + 1).cast<float>()));

        top_left_string_x.setNum(-zero_geo_w.y(), 'f', 10);
        top_left_string_y.setNum(zero_geo_w.x(), 'f', 10);
        out << top_left_string_x << "\n" << top_left_string_y << "\n";

        tfwFile.close();

        if (!success) {
            //ROS_INFO("Writing image with file %s failed with error %s", complete_file_string.c_str(),
            //         imageWriter.errorString().toStdString().c_str());
        } else {
            //ROS_INFO("Successfully wrote geotiff to %s", complete_file_string.c_str());
        }
    }

    void GeotiffWriter::transformPainterToImgCoords(QPainter &painter) {
        painter.rotate(-90);
        painter.translate(-geoTiffSizePixels.x(), geoTiffSizePixels.y());
        painter.scale(1.0, -1.0);
    }

    void GeotiffWriter::drawCoords() {
        QPainter qPainter(&image);
        qPainter.setFont(map_draw_font_);
        float arrowOffset = pixelsPerGeoTiffMeter * 0.15f;

        // MAP ORIENTATION
        qPainter.setPen(QColor(0, 50, 140));
        qPainter.drawLine(pixelsPerGeoTiffMeter / 2, pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter / 2,
                          2.0f * pixelsPerGeoTiffMeter);
        qPainter.drawLine(pixelsPerGeoTiffMeter * 2 / 5, pixelsPerGeoTiffMeter - 1, pixelsPerGeoTiffMeter * 3 / 5,
                          pixelsPerGeoTiffMeter - 1);
        qPainter.drawLine(pixelsPerGeoTiffMeter * 2 / 5, 2 * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter * 3 / 5,
                          2 * pixelsPerGeoTiffMeter);

        qPainter.drawLine(pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter,
                          2 * pixelsPerGeoTiffMeter);
        qPainter.drawLine(pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter + arrowOffset,
                          2 * pixelsPerGeoTiffMeter - arrowOffset);
        qPainter.drawLine(pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter + arrowOffset,
                          2 * pixelsPerGeoTiffMeter + arrowOffset);

        qPainter.drawLine(2 * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter,
                          2 * pixelsPerGeoTiffMeter);
        qPainter.drawLine(2 * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter + arrowOffset,
                          pixelsPerGeoTiffMeter + arrowOffset);
        qPainter.drawLine(2 * pixelsPerGeoTiffMeter, pixelsPerGeoTiffMeter, 2 * pixelsPerGeoTiffMeter - arrowOffset,
                          pixelsPerGeoTiffMeter + arrowOffset);

        qPainter.drawText(0.6 * pixelsPerGeoTiffMeter, 1.6 * pixelsPerGeoTiffMeter, QString("1m"));
        qPainter.drawText(2.2 * pixelsPerGeoTiffMeter, 1.1 * pixelsPerGeoTiffMeter, QString("x"));
        qPainter.drawText(1.2 * pixelsPerGeoTiffMeter, 1.8 * pixelsPerGeoTiffMeter, QString("y"));
        qPainter.drawText(0.5f * pixelsPerGeoTiffMeter, 0.75f * pixelsPerGeoTiffMeter,
                          QString((map_file_name_ + ".tif").c_str()));
    }

    void GeotiffWriter::drawCross(QPainter &painter, const Eigen::Vector2f &coords) {
        painter.drawLine(QPointF(coords[0] - 1.0f, coords[1]), QPointF(coords[0] + 1.0f, coords[1]));
        painter.drawLine(QPointF(coords[0], coords[1] - 1.0f), QPointF(coords[0], coords[1] + 1.0f));
    }

    void GeotiffWriter::drawArrow(QPainter &painter) {
        float tip_distance = pixelsPerGeoTiffMeter * 0.3f;
        QPolygonF polygon;

        polygon << QPointF(tip_distance, 0.0f) << QPointF(-tip_distance * 0.5f, -tip_distance * 0.5f)
                << QPointF(0.0f, 0.0f) << QPointF(-tip_distance * 0.5f, tip_distance * 0.5f);

        painter.save();

        QBrush tmpBrush(QColor(255, 200, 0));
        QPen tmpPen(Qt::NoPen);
        painter.setBrush(tmpBrush);
        painter.setPen(tmpPen);
        painter.drawPolygon(polygon);
        painter.restore();
    }

    void GeotiffWriter::drawCoordSystem(QPainter &painter) {
        painter.save();
        QPointF zero_point(0.0f, 0.0f);
        QPointF x_point(pixelsPerGeoTiffMeter, 0.0f);
        QPointF y_point(0.0f, pixelsPerGeoTiffMeter);

        QPen tmp = painter.pen();
        tmp.setWidth(5);
        tmp.setColor(QColor(255.0, 0.0, 0.0));
        //painter.setPen(QPen::setWidth(5));
        painter.setPen(tmp);
        painter.drawLine(zero_point, x_point);

        tmp.setColor(QColor(0, 255, 0));
        painter.setPen(tmp);
        painter.drawLine(zero_point, y_point);
        painter.restore();
    }

    Eigen::Vector4d GeotiffWriter::interpolateColorMap(const Eigen::MatrixX3d &colorMap, double x) {
        x = std::max(0.0, std::min(1.0, x));
        int length = colorMap.rows() - 1;

        int a = int(x * length);
        int b = std::min(length, a + 1);
        double f = x * length - a;
        Eigen::Vector3d tmp = (colorMap.row(a) + (colorMap.row(b) - colorMap.row(a)) * f);
        return {tmp.x(), tmp.y(), tmp.z(), 1.0};
    }

    Eigen::MatrixX4d
    GeotiffWriter::applyColorMap(const Eigen::MatrixX3d &colorMap, const Vector &values, bool invert, bool logspace) {
        if (colorMap.rows() < 2) {
            STREAM_DEBUG(
                    "ColorMap does not have enough samples, it needs 2 or more but currently has " << colorMap.rows());
            return {};
        }

        Vector val_c = values;
        if (logspace) {
            for (int i = 0; i < values.size(); i++) {
                if (values(i) > 0) {
                    val_c(i) = log(values(i) + 1);
                } else {
                    val_c(i) = 0;
                }
            }
            STREAM_DEBUG("Applied logspace");
        }

        // normalize values to be between 0 and 1
        double min = val_c.minCoeff();
        double max = val_c.maxCoeff();
        Vector values_norm;
        if (min == max) {
            values_norm.setZero(val_c.size());
        } else {
            values_norm = (val_c.array() - min) * (1 / (max - min));
        }
        if (invert) {
            values_norm = 1 - values_norm.array();
        }

        //interpolate values
        int length = colorMap.rows() - 1;
        Vector capped = values_norm.array().max(0.0).min(1.0);

        Eigen::VectorXi a = (capped * length).cast<int>();
        Eigen::VectorXi b = (a.array() + 1).min(length);
        Vector f = capped * length - a.cast<double>();

        Eigen::Vector3d tmp;
        Eigen::MatrixX4d Res(values_norm.size(), 4);
        Res.conservativeResize(values_norm.size(), 4);
        for (int i = 0; i < values_norm.size(); i++) {
            tmp = (colorMap.row(a[i]) + (colorMap.row(b[i]) - colorMap.row(a[i])) * f[i]);
            Res.row(i) << tmp.transpose(), 1.0;
        }
        return Res;
    }

    void GeotiffWriter::setUnit(const std::string &unit) {
        this->unit_ = unit;
    }

    std::string GeotiffWriter::doubleToStr(double value, int digits) {
        std::string value_str = std::to_string(value);
        return value_str.substr(0, value_str.find(',') + digits + 1);
    }
}