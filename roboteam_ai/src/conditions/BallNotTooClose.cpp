//
// Created by baris on 25-4-19.
//

#include "BallNotTooClose.h"
rtt::ai::BallNotTooClose::BallNotTooClose(string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {
    if (properties->getDouble("distance") > 0.02) {
        distance = properties->getDouble("distance");
    }
}
rtt::ai::Condition::Status rtt::ai::BallNotTooClose::onUpdate() {
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    if ((robot->pos - ballPos).length() <= distance) {
        return Status::Failure;
    }
    return Status::Success;
}
