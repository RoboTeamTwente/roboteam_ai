//
// Created by mrlukasbos on 6-12-18.
//

#include "Halt.h"

namespace rtt {
namespace ai {

Halt::Halt(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void Halt::onInitialize() {
    if (properties->hasInt("ticks")) {
        ticks = properties->getInt("ticks");
    }
    tick = 0;
}

Halt::Status Halt::onUpdate() {
    if (tick <= ticks) {
        // send empty command
        command.use_angle = 1;
        publishRobotCommand();
        tick ++;
        return Status::Running;
    }

    return Status::Success;
}

} // ai
} // rtt