//
// Created by rolf on 12-4-19.
//

#include "ShouldHandleBall.h"
#include "../coach/GetBallCoach.h"

namespace rtt {
namespace ai {

ShouldHandleBall::ShouldHandleBall(string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {
}

std::string ShouldHandleBall::node_name() { return "ShouldHandleBall"; }

ShouldHandleBall::Status ShouldHandleBall::onUpdate() {
    if (coach::getBallCoach->getBallGetterID() == robot->id) {
        return Status::Success;
    }
    return Status::Failure;
}

}//ai
}//rtt

