#include "ros/ros.h"

#include "../ros_handler.h"
#include "../world/filtered_world.h"


int main(int argc, char **argv)
{
    // Init ros.
    ros::init(argc, argv, "filtered_world");

    rtt::FilteredWorld world;

    rtt::RosHandler handler;
    handler.init(&world);

    ROS_INFO("---- Filtered world ready. ----");

    ros::spin();

    return 0;
}
