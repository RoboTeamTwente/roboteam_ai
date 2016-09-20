#pragma once

#include "ros/ros.h"

#include "roboteam_world/World.h"

#include "world_dummy.h"


class RosHandler {

private:
    ros::NodeHandle n;
    ros::Subscriber vision_sub;
    ros::Publisher world_pub;

    WorldDummy* world;

public:
    RosHandler();
    void init(WorldDummy* _world);

    void detection_callback(const roboteam_vision::DetectionFrame msg);
};
