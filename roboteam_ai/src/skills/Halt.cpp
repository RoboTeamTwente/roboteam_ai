//
// Created by mrlukasbos on 6-12-18.
//

#include "Halt.h"

namespace rtt {
namespace ai {

/// Example of a Skill with no features - DO NOT EDIT
Halt::Halt(string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) { }

/// Return name of the skill
std::string Halt::node_name() {
    return "Halt";
}

void Halt::initialize() {
    robot = getRobotFromProperties(properties);
}

/// Called when the Skill is Updated
Halt::Status Halt::update() {
    updateRobot();

    if (robot) {
        // send empty cmd
        roboteam_msgs::RobotCommand cmd;
        cmd.id = robot->id;
        cmd.x_vel = 0;
        cmd.y_vel = 0;
        cmd.w = 0;
        publishRobotCommand(cmd);

        return Status::Success;
    } else {
        return Status::Failure;
    }
}



} // ai
} // rtt