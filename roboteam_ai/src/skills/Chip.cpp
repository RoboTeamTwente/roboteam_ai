//
// Created by mrlukasbos on 23-10-18.
//

#include "Chip.h"

namespace rtt {
namespace ai {

Chip::Chip(std::string name, bt::Blackboard::Ptr blackboard)
        :Kick(std::move(name), std::move(blackboard)) { }

void Chip::sendKickCommand(double kickVel) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    // TODO check if we can avoid the casting to unsigned char without warnings
    command.chipper = (unsigned char) true;
    command.chipper_forced = (unsigned char) true;
    command.chipper_vel = (float) kickVel;

    publishRobotCommand(command);
}
} // ai
} // rtt