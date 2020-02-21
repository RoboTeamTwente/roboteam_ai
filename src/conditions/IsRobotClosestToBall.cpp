/*
 * Determine the robot closest to ball. return SUCCESS if the robot is closest.
 * properties:
 *  - secondsAhead: the amount of seconds to linearly extrapolate the ball position
 *  - atBallStillPosition: the position where the ball is expected to lay still due to rolling friction
 */

#include "conditions/IsRobotClosestToBall.h"

namespace rtt::ai {

IsRobotClosestToBall::IsRobotClosestToBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)) {}

bt::Node::Status IsRobotClosestToBall::onUpdate() {
    Vector2 ballPos;
    if (properties->getBool("atBallStillPosition")) {
        ballPos = ball->get()->getExpectedEndPosition();
    } else if (properties->hasDouble("secondsAhead")) {
        double t = properties->getDouble("secondsAhead");
        ballPos = ball->get()->getPos() + ball->get()->getVelocity() * t;
    } else {
        ballPos = ball->get()->getPos();
    }

    auto robotClosestToBall = world->getRobotClosestToPoint(ballPos, rtt::world_new::us);
    if (robotClosestToBall && robotClosestToBall->getId() == robot->get()->getId()) {
        return Status::Success;
    }

    return Status::Failure;
}

}  // namespace rtt::ai