//
// Created by baris on 5-12-18.
//

#include "Pass.h"
namespace rtt {
namespace ai {

Pass::Pass(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}

/// Return name of the skill
std::string Pass::node_name() {
    return "Pass";
}

/// Called when the Skill is Initialized
void Pass::initialize() {
    robot = getRobotFromProperties(properties);

    defensive = properties->getBool("defensive");
    robotToPass = -1;
}

/// Called when the Skill is Updated
Pass::Status Pass::update() {
    updateRobot();

    if (robotToPass == -1) {
        robotToPass = getRobotToPass();
        return Status::Running;
    }
    if (sendPassCommand()) {
        return Status::Success;
    }

    return Status::Running;
}


bool Pass::sendPassCommand() {

    /*
     * Try to pass tp the given robot. If it is not possible at the moment return false
     */


    return false;
}
int Pass::getRobotToPass() {
    return 0;
}

}
}
