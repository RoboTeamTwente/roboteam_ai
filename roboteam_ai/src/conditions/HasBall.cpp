//
// Created by rolf on 19-10-18.
//

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

    if (! robot || ! ball) return Status::Failure;
    if (botHasBall(ball->pos)) return Status::Success;
    else return Status::Failure;
}

bool HasBall::botHasBall(Vector2 ballPos) {
    return (Control::hasBall(constants::MAX_BALL_BOUNCE_RANGE,robot->w,robot->pos,ballPos));
}

} // ai
} // rtt