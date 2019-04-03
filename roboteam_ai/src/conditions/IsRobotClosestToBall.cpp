//
// Created by robzelluf on 10/18/18.
//

#include "IsRobotClosestToBall.h"
#include "../world/World.h"

namespace rtt {
namespace ai {

IsRobotClosestToBall::IsRobotClosestToBall(std::string name, bt::Blackboard::Ptr blackboard)
: Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status IsRobotClosestToBall::onUpdate() {
    auto w = world::world->getWorld();
    BallPtr ball = world::world->getBall();
    Vector2 ballPos(w.ball.pos);

    if (properties->hasDouble("secondsAhead")) {
        double t = properties->getDouble("secondsAhead");
        Vector2 ballVel(w.ball.vel);
        ballPos += ballVel * t;
    }

    Robot robotClosestToBallPtr = world::world->getRobotClosestToBall(world::OUR_ROBOTS);

    if (robotClosestToBallPtr.id == robot->id)
        return Status::Success;

    return Status::Failure;
}

} // ai
} // rtt