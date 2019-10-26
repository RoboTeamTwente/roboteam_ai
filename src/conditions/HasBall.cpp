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
#include "world/World.h"
#include "world/Robot.h"

namespace rtt {
namespace ai {

HasBall::HasBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), blackboard) { }

bt::Node::Status HasBall::onUpdate() {
    return world::world->ourRobotHasBall(robot->id) ? Status::Success : Status::Failure;
}

} // ai
} // rtt