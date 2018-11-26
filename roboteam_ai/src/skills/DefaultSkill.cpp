//
// Created by thijs on 19-11-18.
//

#include "DefaultSkill.h"

namespace rtt {
namespace ai {

/// Example of a Skill with no features - DO NOT EDIT
DefaultSkill::DefaultSkill(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}

/// Return name of the skill
std::string DefaultSkill::node_name() {
    return "DefaultSkill";
}

/// Called when the Skill is Initialized
void DefaultSkill::initialize() {

    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        }
        else {
            ROS_ERROR("DefaultSkill Initialize -> robot does not exist in world");
            return;
        }
    }
    else {
        ROS_ERROR("DefaultSkill Initialize -> ROLE INVALID!!");
        return;
    }
//  ____________________________________________________________________________________________________________________

    variable1 = properties->getBool("goToBall");

}

/// Called when the Skill is Updated
DefaultSkill::Status DefaultSkill::update() {

    if (World::getRobotForId(robot.id, true)) {
        robot = World::getRobotForId(robot.id, true).get();
    } else {
        ROS_ERROR("DefaultSkill Update -> robot does not exist in world");
    }
//  ____________________________________________________________________________________________________________________

    if (variable1) {
        // do things
    }

//  ____________________________________________________________________________________________________________________
    return Status::Success;
}

/// Called when the Skill is Terminated
void DefaultSkill::terminate(Status s) {

}

} // ai
} // rtt