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

    // 2D model
    double circleRadius;
    double minDistanceBetweenSamples2d;
    int minUpdateTime2d;

    // GridMap
    double gridMapResolution;

    // 3D model
    double gpLocalRadius;
    double minDistanceBetweenSamples3d;
    int minUpdateTime3d;

    // 3D Point cloud
    std::vector<double> distanceCutoffLevels;
    std::vector<int> pointCloud3DSizeLevels;

private:
    /**
     * Constructor for the Parameters class.
     * Loads all parameters from the parameter server.
     */
    Parameters() {
        nodeHandle_ = std::make_shared<ros::NodeHandle>("hector_radiation_mapping");

        bool success = true;
        startTime_ = ros::Time::now().toSec();

        doseSubSize = 100;

        // General
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/rosSpinnerThreads", rosSpinnerThreads);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/enableOnline3DEvaluation", enableOnline3DEvaluation);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/enableSpatialSampleFiltering", enableSpatialSampleFiltering);

        // ROS topics
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/subscribeTopic", subscribeTopic);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/environmentMapTopic", environmentMapTopic);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/environmentCloudTopic", environmentCloudTopic);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/messageKey_rate", messageKey_rate);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/messageKey_cps", messageKey_cps);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/messageKey_frameId", messageKey_frameId);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/radiationUnit", radiationUnit);

        // Source prediction
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/meanFactor", meanFactor);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/minSourceStrength", minSourceStrength);

        // 2D model
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/circleRadius", circleRadius);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/minDistanceBetweenSamples2d", minDistanceBetweenSamples2d);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/minUpdateTime2d", minUpdateTime2d);

        // GridMap
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/gridMapResolution", gridMapResolution);

        // 3D model
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/gpLocalRadius", gpLocalRadius);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/minDistanceBetweenSamples3d", minDistanceBetweenSamples3d);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/minUpdateTime3d", minUpdateTime3d);

        // PointCloud3D
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/distanceCutoffLevels", distanceCutoffLevels);
        success = success & nodeHandle_->getParam("/hector_radiation_mapping/pointCloud3DSizeLevels", pointCloud3DSizeLevels);

        if (!success) {
            ROS_ERROR("Could not load all parameters!");
        }

        // Set useDoseRate to true, if messageKey_rate is set
        useDoseRate = !messageKey_rate.empty();

        exportPath = "/../exports/";
    }

    Parameters(const Parameters &) = delete;
    Parameters &operator=(const Parameters &) = delete;
};

#endif //RADIATION_MAPPING_PARAMETERS_H