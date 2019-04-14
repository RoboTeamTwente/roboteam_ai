/*
* return SUCCESS when the ball is close to a point
* properties:
* - margin: the distance to the ball in which it is 'close'
* - ball: wether to determine the robot is close to the ball
* - position: whether to determine wheter to determine the robot is close to a position (needs ball to be false)
*/

#include "IsCloseToPoint.h"

namespace rtt {
namespace ai {

IsCloseToPoint::IsCloseToPoint(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void IsCloseToPoint::onInitialize() {
    if (properties->hasDouble("margin")) {
        margin = properties->getDouble("margin");
    }

    if (properties->getBool("ball")) {
        position = ball->pos;
    } else {
        position = properties->getVector2("position");
    }
}

IsCloseToPoint::Status IsCloseToPoint::onUpdate() {
    double deltaPos = (position - robot->pos).length();
    if (deltaPos >= margin) {
        return Status::Failure;
    }
    return Status::Success;
}
}
}
