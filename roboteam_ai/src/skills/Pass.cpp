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

    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        }
        else {
            ROS_ERROR("Pass Initialize -> robot does not exist in world");
            return;
        }
    }
    else {
        ROS_ERROR("Pass Initialize -> ROLE WAITING!!");
        return;
    }
//  ____________________________________________________________________________________________________________________

    defensive = properties->getBool("defensive");
    robotToPass = -1;
}

/// Called when the Skill is Updated
Pass::Status Pass::update() {

    if (World::getRobotForId(robot.id, true)) {
        robot = World::getRobotForId(robot.id, true).get();
    }
    else {
        ROS_ERROR("Pass Update -> robot does not exist in world");
    }
//  ____________________________________________________________________________________________________________________


    if (robotToPass == -1) {
        robotToPass = getRobotToPass();
        return Status::Running;
    }
    if (sendPassCommand()) {
        return Status::Success;
    }

//  ____________________________________________________________________________________________________________________
    return Status::Running;
}

/// Called when the Skill is Terminated
void Pass::terminate(Status s) {

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
