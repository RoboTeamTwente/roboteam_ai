//
// Created by robzelluf on 6/5/19.
//

#include <include/roboteam_ai/utilities/Constants.h>
#include "include/roboteam_ai/skills/Wait.h"
#include "include/roboteam_ai/world/Robot.h"

namespace rtt {
namespace ai {

Wait::Wait(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void Wait::onInitialize() {
    if(properties->getBool("penalty")) {
        lockedAngle = 0.0;
    } else {
        lockedAngle = robot->angle;
    }

    double seconds;
    if (properties->hasDouble("seconds")) {
        seconds = properties->getDouble("seconds");
    } else {
        seconds = 0;
    }

    ticks = int(seconds * Constants::TICK_RATE());
    tick = 0;
}

Wait::Status Wait::onUpdate() {
    command.set_w(lockedAngle);
    command.set_geneva_state(0);
    publishRobotCommand();
    tick++;
    if(tick>=ticks) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

}
}
