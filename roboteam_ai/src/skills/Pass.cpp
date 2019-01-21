//
// Created by baris on 5-12-18.
//

#include "Pass.h"

namespace rtt {
namespace ai {

Pass::Pass(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

/// Called when the Skill is Initialized
void Pass::onInitialize() {
    defensive = properties->getBool("defensive");
    robotToPass = - 1;
    newTarget = false;
    amIClosest = false;
}

/// Called when the Skill is Updated
Pass::Status Pass::onUpdate() {
    if (! robot)
        return Status::Running;

    PassState passState = Coach::getPassState(properties->getString("ROLE"));

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    Vector2 velocity;


}

} // ai
} // rtt
