//
// Created by mrlukasbos on 23-10-18.
//

#include "Kick.h"

namespace rtt {
namespace ai {

Kick::Kick(std::string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }

void Kick::initialize() {
    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        }
        else {
            ROS_ERROR("GoToPos Initialize -> robot does not exist in world");
            currentProgress = Progression::FAIL;
            return;
        }
    }
    else {
        ROS_ERROR("GoToPos Initialize -> ROLE INVALID!!");
        currentProgress = Progression::FAIL;
        return;
    }
    amountOfCycles = 0;
}

bt::Node::Status Kick::update() {
    // Fail if we did not succeed after a number of cycles
    amountOfCycles ++;
    if (amountOfCycles > constants::MAX_KICK_CYCLES) {
        return Status::Failure;
    }

    // Get kickVelocity from blackboard, otherwise it is a default value.
    double kickVel = properties->hasDouble("kickVel") ? properties->getDouble("kickVel") : constants::DEFAULT_KICK_POWER;

    // Send the robotCommand.
    sendKickCommand(kickVel);
    std::cerr << "        kicking..." << std::endl;
    return Status::Running;
}

void Kick::terminate(status s) {
//
//    roboteam_msgs::RobotCommand command;
//    command.id = robot.id;
//    // TODO check if we can avoid the casting to unsigned char without warnings
//    command.kicker = (unsigned char) false;
//    command.kicker_forced = (unsigned char) false;
//    command.kicker_vel = (float) 0.0f;
//
//    publishRobotCommand(command);

}

void Kick::sendKickCommand(double kickVel) {
    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    // TODO check if we can avoid the casting to unsigned char without warnings
    command.kicker = (unsigned char) true;
    command.kicker_forced = (unsigned char) true;
    command.kicker_vel = (float) kickVel;

    publishRobotCommand(command);
}

} // ai
} // rtt