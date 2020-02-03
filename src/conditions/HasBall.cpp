/*
 *  Returns Success:
 *  - if the robot has the ball
 *
 *  Returns Failure:
 *  - if robot does not have the ball
 *  - if robot or ball is undefined
 */

#include "conditions/HasBall.hpp"

#include <control/ControlUtils.h>

#include "world/Robot.h"
#include "world/World.h"

namespace rtt::ai {

    HasBall::HasBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), blackboard) {}

    bt::Node::Status HasBall::onUpdate() { return world->ourRobotHasBall(robot->id) ? Status::Success : Status::Failure; }

}  // namespace rtt::ai