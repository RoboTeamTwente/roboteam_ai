/*
 * returns SUCCESS if a robot should pass or if a robot should get the ball. Otherwise FAILURE.
 */

#include "conditions/ShouldHandleBall.h"
#include "coach/GetBallCoach.h"

namespace rtt::ai {

ShouldHandleBall::ShouldHandleBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)) {}

std::string ShouldHandleBall::node_name() { return "ShouldHandleBall"; }

ShouldHandleBall::Status ShouldHandleBall::onUpdate() {
    bool passExists = coach::g_pass.getRobotBeingPassedTo() != -1;
    if (passExists) {
        if (!coach::g_pass.isPassed() && coach::g_pass.getRobotPassing() == robot->get()->getId()) {
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }

    if (!passExists && coach::getBallCoach->getBallGetterID() == robot->get()->getId()) {
        return Status::Success;
    }

    return Status::Failure;
}

void ShouldHandleBall::onTerminate(Condition::Status s) { Condition::onTerminate(s); }

}  // namespace rtt::ai
