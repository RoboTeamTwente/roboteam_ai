//
// Created by baris on 25-4-19.
//

#include "world_old/Ball.h"
#include "world_old/World.h"
#include "world_old/Robot.h"
#include "conditions/BallIsClose.h"

rtt::ai::BallIsClose::BallIsClose(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {
    if (properties->getDouble("distance") > 0.02) {
        distance = properties->getDouble("distance");
    }
}
rtt::ai::Condition::Status rtt::ai::BallIsClose::onUpdate() {
    Vector2 ballPos = world->getBall()->getPos();
    if ((robot->pos - ballPos).length() <= distance) {
        return Status::Success;
    }
    return Status::Failure;
}
