//
// Created by robzelluf on 12/17/18.
//

#include <roboteam_ai/src/io/IOManager.h>
#include "ControlKick.h"

namespace control {

void ControlKick::kick(ControlKick::RobotPtr& robot) {
    if (!robot) return;
}

void ControlKick::kick(ControlKick::RobotPtr& robot, unsigned char kicker_forced) {
    if (!robot) return;
    kick(robot, kicker_forced, rtt::ai::constants::MAX_KICK_POWER);
}

void ControlKick::kick(ControlKick::RobotPtr& robot, unsigned char kicker_forced, double kicker_vel) {
    if (!robot) return;
    if (kicker_vel > rtt::ai::constants::MAX_KICK_POWER) kicker_vel = rtt::ai::constants::MAX_KICK_POWER;

    Command command;
    command.id = robot->id;
    command.kicker = static_cast<unsigned char>(true);
    command.kicker_vel = static_cast<float>(kicker_vel);
    command.kicker_forced = kicker_forced;

    publishRobotCommand(command);
}

void ControlKick::publishRobotCommand(roboteam_msgs::RobotCommand &command) {
    ioManager.publishRobotCommand(command);
}

ControlKick::ControlKick() {
    rtt::ai::io::IOManager temp(false, true);
    ioManager = temp;
}

}
