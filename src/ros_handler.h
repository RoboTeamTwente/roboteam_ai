#pragma once

#include "ros/ros.h"

#include "roboteam_msgs/World.h"

#include "world/world_dummy.h"


class RosHandler {

private:
    ros::NodeHandle n;
    ros::Subscriber vision_sub;
    ros::Publisher world_pub;

    rtt::WorldBase* world;

public:
    RosHandler();
    void init(rtt::WorldBase* _world);

    void detection_callback(const roboteam_vision::DetectionFrame msg);
};
