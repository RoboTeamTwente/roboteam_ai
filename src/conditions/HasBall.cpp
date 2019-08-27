/* 
 *  Returns Success:
 *  - if the robot has the ball
 *
 *  Returns Failure:
 *  - if robot does not have the ball
 *  - if robot or ball is undefined
 */

#include <roboteam_ai/src/control/ControlUtils.h>
#include "include/roboteam_ai/conditions/HasBall.hpp"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "include/roboteam_ai/world/World.h"
#include "include/roboteam_ai/world/Robot.h"

namespace rtt {
namespace ai {

HasBall::HasBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), blackboard) { }

bt::Node::Status HasBall::onUpdate() {
    return world::world->ourRobotHasBall(robot->id) ? Status::Success : Status::Failure;
}

} // ai
} // rtt