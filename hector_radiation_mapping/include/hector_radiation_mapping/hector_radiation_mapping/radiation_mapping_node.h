#ifndef RADIATION_MAPPING_RADIATION_MAPPING_NODE_H
#define RADIATION_MAPPING_RADIATION_MAPPING_NODE_H

#include "pch.h"

#include "hector_radiation_mapping_msgs/ResetService.h"
#include "hector_radiation_mapping_msgs/ResetServiceRequest.h"
#include "hector_radiation_mapping_msgs/ResetServiceResponse.h"
#include <std_msgs/String.h>

class RadiationMapper {
public:
    /**
     * Constructor for the node with the initiation procedure of all systems
     */
    RadiationMapper();

    /**
     * Shutdown procedure for the node
     */
    static void sigintHandler(int sig);

    /**
     * Callback for the reset service. It resets the models.
     * @param req The request.
     * @param res The response.
     * @return True.
     */
    static bool resetServiceCallback(hector_radiation_mapping_msgs::ResetService::Request &req,
                                     hector_radiation_mapping_msgs::ResetService::Response &res);

    /**
     * Callback for the syscommand topic. It resets the models if the message is "reset".
     * @param msg The message.
     */
    static void sysCmdCallback(const std_msgs::String::ConstPtr &msg);

private:

    ros::ServiceServer reset_service_;
    ros::Subscriber sys_cmd_sub_;
};

#endif //RADIATION_MAPPING_RADIATION_MAPPING_NODE_H