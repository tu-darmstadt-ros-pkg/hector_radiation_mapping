#ifndef RADIATION_MAPPING_PARAMETERS_H
#define RADIATION_MAPPING_PARAMETERS_H

class Parameters {
public:
    static Parameters &instance() {
        static Parameters instance;
        return instance;
    }

    std::shared_ptr<ros::NodeHandle> nodeHandle_;
    double startTime_;

    // General
    int doseSubSize;
    int rosSpinnerThreads;
    bool enableOnline3DEvaluation;
    std::string exportPath;
    std::string worldFrame;
    bool enableSpatialSampleFiltering;

    // ROS topics
    bool useDoseRate;
    std::string subscribeTopic;
    std::string environmentCloudTopic;
    std::string environmentMapTopic;
    std::string messageKey_rate;
    std::string messageKey_cps;
    std::string messageKey_frameId;
    std::string radiationUnit;

    // Source prediction
    double meanFactor;
    double minSourceStrength;

    // Gaussian Process
    // 2D model
    double circleRadius;
    double minDistanceBetweenSamples2d;
    int minUpdateTime2d;
    // GridMap
    std::string gpGridMapTopic;
    double gridMapResolution;
    // 3D model
    double gpLocalRadius;
    double minDistanceBetweenSamples3d;
    int minUpdateTime3d;
    // 3D Point cloud
    std::vector<double> distanceCutoffLevels;
    std::vector<int> pointCloud3DSizeLevels;

    //

private:

    // templated method for loading a parameter from the parameter server
    template<typename T>
    bool loadParam(const std::string &paramName, T &param, const T &defaultValue = T()) {
        std::string completeName = "/hector_radiation_mapping/" + paramName;
        if (!nodeHandle_->param(completeName, param, defaultValue)) {
            ROS_ERROR("Could not load parameter %s using default value", completeName.c_str());
            return false;
        }
        return true;
    }

    /**
     * Constructor for the Parameters class.
     * Loads all parameters from the parameter server.
     */
    Parameters() {
        nodeHandle_ = std::make_shared<ros::NodeHandle>("hector_radiation_mapping");

        startTime_ = ros::Time::now().toSec();
        doseSubSize = 100;

        // General
        loadParam("rosSpinnerThreads", rosSpinnerThreads);
        loadParam("enableOnline3DEvaluation", enableOnline3DEvaluation);
        loadParam("enableSpatialSampleFiltering", enableSpatialSampleFiltering);
        loadParam("exportPath", exportPath);
        loadParam("worldFrame", worldFrame);

        // ROS topics
        loadParam("subscribeTopic", subscribeTopic);
        loadParam("environmentMapTopic", environmentMapTopic);
        loadParam("environmentCloudTopic", environmentCloudTopic);
        loadParam("messageKey_rate", messageKey_rate);
        loadParam("messageKey_cps", messageKey_cps);
        loadParam("messageKey_frameId", messageKey_frameId);
        loadParam("radiationUnit", radiationUnit);

        // Source prediction
        loadParam("meanFactor", meanFactor);
        loadParam("minSourceStrength", minSourceStrength);

        // Gaussian Process
        // 2D model
        loadParam("circleRadius", circleRadius);
        loadParam("minDistanceBetweenSamples2d", minDistanceBetweenSamples2d);
        loadParam("minUpdateTime2d", minUpdateTime2d);
        // GridMap
        loadParam("gpGridMapTopic", gpGridMapTopic);
        loadParam("gridMapResolution", gridMapResolution);
        // 3D model
        loadParam("gpLocalRadius", gpLocalRadius);
        loadParam("minDistanceBetweenSamples3d", minDistanceBetweenSamples3d);
        loadParam("minUpdateTime3d", minUpdateTime3d);
        // PointCloud3D
        loadParam("distanceCutoffLevels", distanceCutoffLevels);
        loadParam("pointCloud3DSizeLevels", pointCloud3DSizeLevels);

        //

        // Set useDoseRate to true, if messageKey_rate is set
        useDoseRate = !messageKey_rate.empty();
    }

    Parameters(const Parameters &) = delete;
    Parameters &operator=(const Parameters &) = delete;
};

#endif //RADIATION_MAPPING_PARAMETERS_H