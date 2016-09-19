#include "ros/ros.h"

#include "RosHandler.h"
#include "WorldDummy.h"


int main(int argc, char **argv)
{
    // Init ros.
    ros::init(argc, argv, "dummy_world");

    WorldDummy world;

    RosHandler handler;
    handler.init(&world);

    ROS_INFO("Dummy world ready.");

    ros::spin();

    return 0;
}
