//
// Created by robzelluf on 10/18/18.
//

#include "IsRobotClosestToBall.h"
#include "../world/World.h"

namespace rtt {
namespace ai{

IsRobotClosestToBall::IsRobotClosestToBall(std::string name, bt::Blackboard::Ptr blackboard)
: Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status IsRobotClosestToBall::onUpdate() {
    auto w = world::world->getWorld();
    Vector2 ballPos(w.ball.pos);
    auto robots = w.us;

    if (properties->hasDouble("secondsAhead")) {
        double t_ahead = properties->getDouble("secondsAhead");
        Vector2 ballVel(w.ball.vel);
        ballPos = ballPos + ballVel.scale(t_ahead);
    }

    auto robotClosestToBallPtr = world::world->getRobotClosestToPoint(ballPos, world::OUR_ROBOTS);
    if (robotClosestToBallPtr && robot) {
        if (robot->id == robotClosestToBallPtr->id) {
            return Status::Success;
        } else {
            return Status::Failure;
        }
    }
    return Status::Failure;
}

} // ai
} // rtt