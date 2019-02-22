//
// Created by baris on 21-2-19.
//

#include "GoBehindBall.h"

namespace rtt {
namespace ai {

GoBehindBall::GoBehindBall(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {

}

Skill::Status GoBehindBall::onUpdate() {
    return Status::Success;
}

void GoBehindBall::onInitialize() {
    Skill::onInitialize();
}

void GoBehindBall::onTerminate(Skill::Status s) {
    Skill::onTerminate(s);
}
}
}