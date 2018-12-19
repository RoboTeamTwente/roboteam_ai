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
}

/// Called when the Skill is Updated
Pass::Status Pass::onUpdate() {

    if (robotToPass == - 1) {
        if (defensive) {
            robotToPass = coach::pickDefensivePassTarget(robot->id);
        }
        else {
            robotToPass = coach::pickOffensivePassTarget(robot->id, properties->getString("ROLE"));
        }
        return Status::Running;
    }
    if (sendPassCommand()) {
        return Status::Success;
    }

    return Status::Running;
}

bool Pass::sendPassCommand() {

    if (getReadyToPass()) {
        // TODO send the magik kick command
        return true;
    }

    return false;
}
bool Pass::getReadyToPass() {

    // Probably stop?

    // Turn to the angle

    // return true when the angle and velocity looks fine

    // Make money !



    return false;
}

} // ai
} // rtt
