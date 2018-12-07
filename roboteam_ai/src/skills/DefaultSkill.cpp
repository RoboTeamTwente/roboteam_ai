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
    robot = getRobotFromProperties(properties);
    variable1 = properties->getBool("goToBall");
}

/// Called when the Skill is Updated
DefaultSkill::Status DefaultSkill::update() {
    updateRobot();

    if (variable1) {
        // do things
    }
    return Status::Success;
}


} // ai
} // rtt