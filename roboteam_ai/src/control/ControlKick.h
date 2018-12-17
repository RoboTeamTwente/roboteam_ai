//
// Created by robzelluf on 12/17/18.
//

#ifndef ROBOTEAM_AI_CONTROLKICK_H
#define ROBOTEAM_AI_CONTROLKICK_H


#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_ai/src/utilities/Constants.h>

#include <roboteam_utils/Vector2.h>
#include <roboteam_msgs/WorldRobot.h>
#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_ai/src/io/IOManager.h>

#include "ros/ros.h"
#include "../io/IOManager.h"
#include "../../src/control/ControlUtils.h"
#include "../utilities/Constants.h"

namespace control {

class ControlKick {
public:
    ControlKick();
private:
    using Command = roboteam_msgs::RobotCommand;
    using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;
    rtt::ai::io::IOManager ioManager;

    void publishRobotCommand(roboteam_msgs::RobotCommand &command);
public:
    void kick(RobotPtr& robot);
    void kick(RobotPtr& robot, unsigned char kicker_forced);
    void kick(RobotPtr& robot, unsigned char kicker_forced, double kicker_vel);
};

}

#endif //ROBOTEAM_AI_CONTROLKICK_H
