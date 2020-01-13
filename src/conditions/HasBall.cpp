/* 
 *  Returns Success:
 *  - if the robot has the ball
 *
 *  Returns Failure:
 *  - if robot does not have the ball
 *  - if robot or ball is undefined
 */

#include <control/ControlUtils.h>
#include "conditions/HasBall.hpp"
#include "world_old/World.h"
#include "world_old/Robot.h"

namespace rtt::ai {

HasBall::HasBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), blackboard) { }

bt::Node::Status HasBall::onUpdate() {
    return world->ourRobotHasBall(robot->id) ? Status::Success : Status::Failure;
}

} // rtt