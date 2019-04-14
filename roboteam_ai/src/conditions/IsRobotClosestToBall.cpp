/*
 * Determine the robot closest to ball. return SUCCESS if the robot is closest.
 * properties:
 *  - secondsAhead: the amount of seconds to linearly extrapolate the ball position, (to predict which robot will be closest)
 */ 

#include "IsRobotClosestToBall.h"
#include "../world/World.h"

namespace rtt {
namespace ai {

IsRobotClosestToBall::IsRobotClosestToBall(std::string name, bt::Blackboard::Ptr blackboard)
: Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status IsRobotClosestToBall::onUpdate() {
    Vector2 ballPos(ball->pos);

    if (properties->hasDouble("secondsAhead")) {
        double t = properties->getDouble("secondsAhead");
        Vector2 ballVel = ball->vel;
        ballPos += ballVel * t;
    }

    Robot robotClosestToBall = world::world->getRobotClosestToPoint(ballPos, world::OUR_ROBOTS);
    if (robotClosestToBall.id == robot->id) {
        return Status::Success;
    }

    return Status::Failure;
}

} // ai
} // rtt