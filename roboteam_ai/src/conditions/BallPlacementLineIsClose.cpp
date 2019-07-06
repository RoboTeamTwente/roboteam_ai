//
// Created by roboteam on 6/07/19.
//

#include "BallPlacementLineIsClose.h"

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/world/Ball.h>
#include <roboteam_ai/src/world/Robot.h>
#include <roboteam_ai/src/utilities/GameStateManager.hpp>
#include <roboteam_ai/src/control/ControlUtils.h>

rtt::ai::BallPlacementLineIsClose::BallPlacementLineIsClose(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {
    if (properties->getDouble("distance") > 0.02) {
        distance = properties->getDouble("distance");
    }
}
rtt::ai::Condition::Status rtt::ai::BallPlacementLineIsClose::onUpdate() {
    Vector2 ballPos = rtt::ai::world::world->getBall()->pos;
    Vector2 ballPlacementTarget = rtt::ai::GameStateManager::getRefereeData().designated_position;

    double distToLine = rtt::ai::control::ControlUtils::distanceToLineWithEnds(robot->pos, ballPos, ballPlacementTarget);

    if (distToLine <= distance) {
        return Status::Success;
    }
    return Status::Failure;
}
