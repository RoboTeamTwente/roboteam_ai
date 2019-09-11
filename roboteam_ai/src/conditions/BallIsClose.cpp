//
// Created by baris on 25-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/world/Ball.h>
#include <roboteam_ai/src/world/Robot.h>
#include "BallIsClose.h"

rtt::ai::BallIsClose::BallIsClose(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {
    if (properties->getDouble("distance") > 0.02) {
        distance = properties->getDouble("distance");
    }
}
rtt::ai::Condition::Status rtt::ai::BallIsClose::onUpdate() {
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    if ((robot->pos - ballPos).length() <= distance) {
        return Status::Success;
    }
    return Status::Failure;
}
