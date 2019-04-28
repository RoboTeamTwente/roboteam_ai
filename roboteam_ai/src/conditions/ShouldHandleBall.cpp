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
    if (coach::g_pass.getRobotBeingPassedTo() != -1) {
        if (coach::g_pass.getRobotPassing() == robot->id) {
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }

    if (coach::getBallCoach->getBallGetterID() == robot->id) {
        return Status::Success;
    }

    return Status::Failure;
}
void ShouldHandleBall::onTerminate(Condition::Status s) {
    Condition::onTerminate(s);

    if (s == Status::Failure) {
        if (coach::g_pass.getRobotPassing() == robot->id) {
            coach::g_pass.resetPass();
        }
    }
}

}//ai
}//rtt

