#ifndef RADIATION_MAPPING_MODEL_MANAGER_H
#define RADIATION_MAPPING_MODEL_MANAGER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_msgs/String.h>
#include "custom_geotiff/geotiff_writer.h"

#include "hector_radiation_mapping_msgs/ResetService.h"
#include "hector_radiation_mapping_msgs/ResetServiceRequest.h"
#include "hector_radiation_mapping_msgs/ResetServiceResponse.h"

class PointCloud3D;
class Source;
class GPython2D;
class GPython3D;
class GPython;

class ModelManager {
public:
    static ModelManager& instance();

    /**
     * Resets the models. It deletes all markers and interactive markers.
     */
    void resetModels();

    /**
     * Shuts down the model manager. It shuts down the model exporter and the radiation models.
     */
    void shutDown();

    /**
     * Callback for the reset service. It resets the models.
     * @param req The request.
     * @param res The response.
     * @return True.
     */
    bool resetServiceCallback(hector_radiation_mapping_msgs::ResetService::Request &req,
                              hector_radiation_mapping_msgs::ResetService::Response &res);

    /**
     * Callback for the syscommand topic. It resets the models if the message is "reset".
     * @param msg The message.
     */
    void sysCmdCallback(const std_msgs::String::ConstPtr &msg);

private:
    /**
     * Private constructor for singleton pattern.
     * It initializes the reset service and the syscommand subscriber. It also initializes the model exporter
     * and radiation models.
     */
    ModelManager();
    ModelManager(const ModelManager&) = delete;
    ModelManager& operator=(const ModelManager&) = delete;

    ros::ServiceServer resetService_;
    ros::Subscriber sysCmdSub_;
};

#endif //RADIATION_MAPPING_MODEL_MANAGER_H