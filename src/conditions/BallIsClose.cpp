/*
 * returns SUCCESS if the ball is close (properties->"distance") to the robot. Otherwise FAILURE.
 */
#include "conditions/BallIsClose.h"

rtt::ai::BallIsClose::BallIsClose(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {
    if (properties->getDouble("distance") > 0.02) {
        distance = properties->getDouble("distance");
    }
}
rtt::ai::Condition::Status rtt::ai::BallIsClose::onUpdate() {
    Vector2 ballPos = ball->get()->getPos();
    if ((robot->get()->getPos() - ballPos).length() <= distance) {
        return Status::Success;
    }
    return Status::Failure;
}
