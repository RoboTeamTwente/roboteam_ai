//
// Created by baris on 15-1-19.
//

#include "BasicGoToPos.h"

namespace rtt {
namespace ai {

BasicGoToPos::BasicGoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {

}

void BasicGoToPos::onInitialize() {
    robot = getRobotFromProperties(properties);
}


Skill::Status BasicGoToPos::onUpdate() {
    return Status::Success;
}
}
}
