#include "ros/ros.h"

#include "roboteam_world/ros_handler.h"
#include "roboteam_world/world/filtered_world.h"
#include "roboteam_world/predictor.h"
#include "roboteam_utils/LastWorld.h"

int main(int argc, char **argv)
{

    // Init ros.
    ros::init(argc, argv, "filtered_world");

    //TODO: Fix this geometry hack; initializing a class to subscribe to a ROS topic is bad
    // Create subscribers for world & geom messages
    rtt::WorldAndGeomCallbackCreator cb;

    double memory_time = 0.1;
    rtt::Predictor predictor(memory_time);

    rtt::FilteredWorld world(predictor);

    rtt::RosHandler handler;
    handler.init(&world);

    ROS_INFO("---- Filtered world ready. ----");

    ros::spin();

    return 0;
}
