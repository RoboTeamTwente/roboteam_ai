/*
 * return SUCCESS when the ball is close to a point
 * properties:
 * - margin: the distance to the ball in which it is 'close'
 * - ball: whether to determine the robot is close to the ball
 * - position: whether to determine whether to determine the robot is close to a position (needs ball to be false)
 */

#include "conditions/IsCloseToPoint.h"

namespace rtt::ai {

IsCloseToPoint::IsCloseToPoint(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)){};

void IsCloseToPoint::onInitialize() {
    if (properties->hasDouble("margin")) {
        margin = properties->getDouble("margin");
    }

    if (properties->getBool("ball")) {
        position = ball->get()->getPos();
    } else {
        position = properties->getVector2("position");
    }
}

IsCloseToPoint::Status IsCloseToPoint::onUpdate() {
    double deltaPos = (position - robot->get()->getPos()).length();
    if (deltaPos >= margin) {
        return Status::Failure;
    }
    return Status::Success;
}

}  // namespace rtt::ai
