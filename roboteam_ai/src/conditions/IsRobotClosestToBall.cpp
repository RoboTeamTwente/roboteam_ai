//
// Created by robzelluf on 10/18/18.
//

#include "IsRobotClosestToBall.h"
#include "../utilities/World.h"

namespace rtt {
namespace ai{

IsRobotClosestToBall::IsRobotClosestToBall(std::string name, bt::Blackboard::Ptr blackboard)
: Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status IsRobotClosestToBall::onUpdate() {
    roboteam_msgs::World world = World::get_world();
    Vector2 ballPos(world.ball.pos);
    std::vector<roboteam_msgs::WorldRobot> robots = world.us;

    if (properties->hasDouble("secondsAhead")) {
        double t_ahead = properties->getDouble("secondsAhead");
        Vector2 ballVel(world.ball.vel);
        ballPos = ballPos + ballVel.scale(t_ahead);
    }

    auto robotClosestToBallPtr = World::getRobotClosestToPoint(robots, ballPos);
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