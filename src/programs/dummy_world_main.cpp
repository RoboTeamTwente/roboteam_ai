#include "ros/ros.h"

#include "roboteam_world/ros_handler.h"
#include "roboteam_world/world/world_dummy.h"


int main(int argc, char **argv)
{
    // Init ros.
    ros::init(argc, argv, "dummy_world");

    rtt::WorldDummy world;

    rtt::RosHandler handler;
    handler.init(&world);

    ROS_INFO("---- Dummy world ready. ----");

    ros::spin();

    return 0;
}
