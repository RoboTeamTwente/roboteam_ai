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

/// Called when the Skill is Initialized
void Halt::initialize() {

    if (properties->hasString("ROLE")) {

        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);

        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        } else {
            ROS_ERROR("Halt Initialize -> robot does not exist in world");
            return;
        }
    }
    else {
        ROS_ERROR("Halt Initialize -> ROLE WAITING!!");
        return;
    }
}

/// Called when the Skill is Updated
Halt::Status Halt::update() {
    if (World::getRobotForId(robot.id, true)) {
        robot = World::getRobotForId(robot.id, true).get();
    } else {
        ROS_ERROR("Halt Update -> robot does not exist in world");
    }

    // send empty cmd
    roboteam_msgs::RobotCommand cmd;
    cmd.id = robot.id;
    cmd.x_vel = 0;
    cmd.y_vel = 0;
    cmd.w = 0;
    publishRobotCommand(cmd);

    return Status::Success;
}

/// Called when the Skill is Terminated
void Halt::terminate(Status s) {

}

}
}