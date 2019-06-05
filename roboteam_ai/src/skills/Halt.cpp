//
// Created by mrlukasbos on 6-12-18.
//

#include "Halt.h"

namespace rtt {
namespace ai {

Halt::Halt(std::string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void Halt::onInitialize() {
    if (properties->hasInt("ticks")) {
        ticks = properties->getInt("ticks");
    } else {
        ticks = 0;
    }
    tick = 0;
}

Halt::Status Halt::onUpdate() {
    // send slowing down command
    command.w = robot->angle;
    publishRobotCommand();
    tick ++;

    if (tick <= ticks) {
        return Status::Running;
    } else {
        return Status::Success;
    }
}

} // ai
} // rtt