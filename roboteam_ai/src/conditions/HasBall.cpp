/* 
 *  Returns Success:
 *  - if the robot has the ball
 *
 *  Returns Failure:
 *  - if robot does not have the ball
 *  - if robot or ball is undefined
 */

#include <roboteam_ai/src/control/ControlUtils.h>
#include "HasBall.hpp"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "../utilities/World.h"

namespace rtt {
namespace ai {

HasBall::HasBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) { }

bt::Node::Status HasBall::onUpdate() {
    if (! robot || ! ball) {
        return Status::Failure;
    }
    if (World::botHasBall(robot->id,true)) {
        return Status::Success;
    }
    return Status::Failure;
}
} // ai
} // rtt