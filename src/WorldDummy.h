#pragma once

#include <map>

#include "ros/ros.h"

#include "roboteam_vision/DetectionFrame.h"
#include "roboteam_world/World.h"

#include "Robot.h"
#include "Ball.h"


typedef std::map<uint, rtt::Robot> RobotMap;


class WorldDummy {

private:
    RobotMap robots_yellow;
    RobotMap robots_blue;
    rtt::Ball ball;

public:
    WorldDummy();
    void detection_callback(const roboteam_vision::DetectionFrame msg);

    roboteam_world::World as_message();
};
