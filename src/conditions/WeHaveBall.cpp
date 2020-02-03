/*
 * return SUCCESS if one of our robots has the ball, otherwise FAILURE
 */

#include "conditions/WeHaveBall.h"

#include "world/Robot.h"
#include "world/World.h"

namespace rtt::ai {

    WeHaveBall::WeHaveBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)) {}

    bt::Node::Status WeHaveBall::onUpdate() {
        RobotPtr robotThatHasBall = world->whichRobotHasBall();
        if (robotThatHasBall && robotThatHasBall->team == Team::us) {
            return bt::Node::Status::Success;
        }
        return bt::Node::Status::Failure;
    }

}  // namespace rtt::ai