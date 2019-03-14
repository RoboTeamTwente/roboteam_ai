//
// Created by baris on 14-3-19.
//

#include "ShootFreeKick.h"

namespace rtt {
namespace ai {

ShootFreeKick::ShootFreeKick(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}

Skill::Status ShootFreeKick::onUpdate() {
    return Status::Waiting;
}

void ShootFreeKick::onInitialize() {
}

void ShootFreeKick::onTerminate(Skill::Status s) {
}
}
}