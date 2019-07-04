//
// Created by robzelluf on 6/5/19.
//

#include <roboteam_ai/src/utilities/Constants.h>
#include "Wait.h"
#include "../world/Robot.h"

namespace rtt {
namespace ai {

Wait::Wait(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void Wait::onInitialize() {
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
    command.w = robot->angle;
    command.geneva_state=0;
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
