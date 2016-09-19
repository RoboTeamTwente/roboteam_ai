#pragma once

#include "ros/ros.h"

#include "roboteam_world/World.h"

#include "WorldDummy.h"


class RosHandler {

    ros::NodeHandle n;
    ros::Subscriber vision_sub;
    ros::Publisher world_pub;

    WorldDummy* world;

public:
    RosHandler();
    void init(WorldDummy* _world);

    void detectionCallback(const roboteam_vision::DetectionFrame msg);
};
