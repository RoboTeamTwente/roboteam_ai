#include "ros/ros.h"

#include "roboteam_vision/DetectionFrame.h"


void detectionCallback(const roboteam_vision::DetectionFrame msg) {
    ROS_INFO("Got frame [%u] from [%u]", msg.frame_number, msg.camera_id);
}


int main(int argc, char **argv)
{
    // Init ros.
    ros::init(argc, argv, "dummy_world");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("vision_detection", 1000, detectionCallback);

    ros::spin();

    return 0;
}
