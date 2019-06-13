/*
 * Determine the robot closest to ball. return SUCCESS if the robot is closest.
 * properties:
 *  - secondsAhead: the amount of seconds to linearly extrapolate the ball position
 *  - atBallStillPosition: the position where the ball is expected to lay still due to rolling friction
 */ 

#include "IsRobotClosestToBall.h"
#include "../world/World.h"

namespace rtt {
namespace ai {

IsRobotClosestToBall::IsRobotClosestToBall(std::string name, bt::Blackboard::Ptr blackboard)
: Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status IsRobotClosestToBall::onUpdate() {
    Vector2 ballPos;
    if (properties->getBool("atBallStillPosition")) {
        ballPos = ball->getBallStillPosition();
    }
    else if (properties->hasDouble("secondsAhead")) {
        double t = properties->getDouble("secondsAhead");
        ballPos = ball->pos + ball->vel * t;
    }
    else {
        ballPos = ball->pos;
    }

    auto robotClosestToBall = world::world->getRobotClosestToPoint(ballPos, world::OUR_ROBOTS);
    if (robotClosestToBall && robotClosestToBall->id == robot->id) {
        return Status::Success;
    }

    return Status::Failure;
}

} // ai
} // rtt