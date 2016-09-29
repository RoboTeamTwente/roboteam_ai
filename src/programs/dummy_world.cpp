#include "ros/ros.h"

#include "../ros_handler.h"
#include "../world/world_dummy.h"


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
