#pragma once

#include <map>

#include "ros/ros.h"

#include "roboteam_vision/DetectionFrame.h"
#include "roboteam_world/World.h"

#include "Robot.h"


typedef std::map<uint, rtt::Robot> RobotMap;


class WorldDummy {

private:
    RobotMap robots_yellow;
    RobotMap robots_blue;

public:
    WorldDummy();
    void detection_callback(const roboteam_vision::DetectionFrame msg);

    roboteam_world::World get_message();
};
