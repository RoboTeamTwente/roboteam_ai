/*
 * Return SUCCESS if their robots have the ball, otherwise FAILURE
 */

#include "conditions/TheyHaveBall.h"

namespace rtt::ai {

TheyHaveBall::TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)) {}

bt::Node::Status TheyHaveBall::onUpdate() {
    rtt::world_new::view::RobotView robotThatHasBall = world->whichRobotHasBall().value();

    if (robotThatHasBall && robotThatHasBall->getTeam() == rtt::world_new::them) {
        return bt::Node::Status::Success;
    }

    return bt::Node::Status::Failure;
}

}  // namespace rtt::ai