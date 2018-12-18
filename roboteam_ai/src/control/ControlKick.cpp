//
// Created by robzelluf on 12/17/18.
//

#include <roboteam_ai/src/io/IOManager.h>
#include "ControlKick.h"

namespace control {

ControlKick::ControlKick() {
    rtt::ai::io::IOManager temp(false, true);
    ioManager = temp;
}

void ControlKick::kick(ControlKick::RobotPtr &robot) {
    if (! robot) return;
    kick(robot, static_cast<unsigned char>(true), MAX_KICKER_VEL);
}

void ControlKick::kick(ControlKick::RobotPtr &robot, unsigned char kicker_forced) {
    if (! robot) return;
    kick(robot, kicker_forced, MAX_KICKER_VEL);
}

void ControlKick::kick(ControlKick::RobotPtr &robot, unsigned char kicker_forced, float kicker_vel) {
    if (! robot) return;
    if (kicker_vel > MAX_KICKER_VEL) kicker_vel = MAX_KICKER_VEL;

    Command command;
    command.id = robot->id;
    command.kicker = static_cast<unsigned char>(true);
    command.kicker_vel = kicker_vel;
    command.kicker_forced = kicker_forced;

    publishRobotCommand(command);
}

void ControlKick::publishRobotCommand(roboteam_msgs::RobotCommand &command) {
    ioManager.publishRobotCommand(command);
}

}
