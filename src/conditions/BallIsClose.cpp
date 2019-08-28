//
// Created by baris on 25-4-19.
//

#include <include/roboteam_ai/world/World.h>
#include <include/roboteam_ai/world/Ball.h>
#include <include/roboteam_ai/world/Robot.h>
#include "include/roboteam_ai/conditions/BallIsClose.h"

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
