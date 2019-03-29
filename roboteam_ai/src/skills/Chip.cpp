//
// Created by mrlukasbos on 23-10-18.
//

#include "Chip.h"

namespace rtt {
namespace ai {

Chip::Chip(std::string name, bt::Blackboard::Ptr blackboard)
        :Kick(std::move(name), std::move(blackboard)) { }

void Chip::sendKickCommand(double kickVel) {

    command.chipper = (unsigned char) true;
    command.chipper_forced = (unsigned char) true;
    command.chipper_vel = (float) kickVel;

    publishRobotCommand();
}
} // ai
} // rtt