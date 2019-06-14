//
// Created by rolf on 12-4-19.
//

#include "ShouldHandleBall.h"
#include "../coach/GetBallCoach.h"
#include "roboteam_ai/src/world/Robot.h"

namespace rtt {
namespace ai {

ShouldHandleBall::ShouldHandleBall(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) {
}

std::string ShouldHandleBall::node_name() { return "ShouldHandleBall"; }

ShouldHandleBall::Status ShouldHandleBall::onUpdate() {
    bool passExists = coach::g_pass.getRobotBeingPassedTo() != - 1 && ! coach::g_pass.isPassed();
    if (passExists && coach::g_pass.getRobotPassing() == robot->id) {
        return Status::Success;
    }

    if (! passExists && coach::getBallCoach->getBallGetterID() == robot->id) {
        return Status::Success;
    }

    return Status::Failure;
}

void ShouldHandleBall::onTerminate(Condition::Status s) {
    Condition::onTerminate(s);

}

}//ai
}//rtt

