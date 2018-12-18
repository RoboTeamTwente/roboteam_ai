//
// Created by mrlukasbos on 23-10-18.
//

#include "Kick.h"

namespace rtt {
namespace ai {

Kick::Kick(std::string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void Kick::onInitialize() {
    amountOfCycles = 0;
}

bt::Node::Status Kick::onUpdate() {
    // Fail if we did not succeed after a number of cycles
    amountOfCycles ++;
    if (amountOfCycles > constants::MAX_KICK_CYCLES) {
        return Status::Failure;
    }

    // Get kickVelocity from blackboard, otherwise it is a default value.
    double kickVel = properties->hasDouble("kickVel") ? properties->getDouble("kickVel")
                                                      : constants::DEFAULT_KICK_POWER;
    sendKickCommand(kickVel);
    return Status::Running;
}

void Kick::sendKickCommand(double kickVel) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.kicker = (unsigned char) true;
    command.kicker_forced = (unsigned char) true;
    command.kicker_vel = (float) kickVel;
    publishRobotCommand(command);
}

} // ai
} // rtt