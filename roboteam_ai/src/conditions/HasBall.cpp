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

bt::Node::Status HasBall::update() {
    robot = getRobotFromProperties(properties);
    auto ball = World::getBall();

    if (properties->hasDouble("ballRange")) {
        ballRange = properties->getDouble("ballRange");
    }

    if (! robot || ! ball) {
        return Status::Failure;
    }
    // TODO: Check where this is used, currently optimal for kicking the ball
    if (World::ourBotHasBall(robot->id, ballRange)) {
        return Status::Success;
    }
    return Status::Failure;
}
} // ai
} // rtt