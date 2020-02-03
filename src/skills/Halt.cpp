//
// Created by mrlukasbos on 6-12-18.
//

#include "skills/Halt.h"

namespace rtt::ai {

    Halt::Halt(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

    Halt::Status Halt::onUpdate() {
        // send slowing down command
        command.set_geneva_state(3);
        publishRobotCommand();
        return Status::Running;
    }

}  // namespace rtt::ai