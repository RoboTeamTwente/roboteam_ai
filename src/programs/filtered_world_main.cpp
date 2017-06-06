#include "ros/ros.h"

#include "roboteam_world/ros_handler.h"
#include "roboteam_world/world/filtered_world.h"
#include "roboteam_world/predictor.h"
#include "roboteam_world/danger_finder/DangerFinder.h"


int main(int argc, char **argv)
{
    // Init ros.
    ros::init(argc, argv, "filtered_world");

    // Create subscribers for world & geom messages
    rtt::WorldAndGeomCallbackCreator cb;

    double memory_time = 0.5;
    rtt::Predictor predictor(memory_time);

    rtt::FilteredWorld world(predictor);

    rtt::RosHandler handler;
    handler.init(&world);

    rtt::df::DangerFinder::world = &world;
    rtt::df::DangerFinder::instance().start();

    ROS_INFO("---- Filtered world ready. ----");

    ros::spin();

    return 0;
}
