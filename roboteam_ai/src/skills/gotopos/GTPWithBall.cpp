#include <utility>

#include <utility>

//
// Created by thijs on 26-4-19.
//

#include "GTPWithBall.h"

namespace rtt {
namespace ai {

GTPWithBall::GTPWithBall(string name, bt::Blackboard::Ptr blackboard)
        :GoToPos(std::move(name), std::move(blackboard)) { }

void GTPWithBall::gtpInitialize() {

}

Skill::Status GTPWithBall::gtpUpdate() {



    return Status::Failure;
}

void GTPWithBall::gtpTerminate(Skill::Status s) {

}

}
}