/*
 * return SUCCESS if one of our robots has the ball, otherwise FAILURE
 */

#include "conditions/WeHaveBall.h"

namespace rtt::ai {

WeHaveBall::WeHaveBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)) {}

bt::Node::Status WeHaveBall::onUpdate() {
    rtt::world_new::view::RobotView robotThatHasBall = world.whichRobotHasBall().value();

    if (robotThatHasBall && robotThatHasBall->getTeam() == rtt::world_new::us) {
        return bt::Node::Status::Success;
    }
    return bt::Node::Status::Failure;
}

}  // namespace rtt::ai