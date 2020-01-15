//
// Created by jessevw on 06.12.19.
//

#include <roboteam_utils/Vector2.h>
#include "world_old/World.h"
#include "world_old/Field.h"
#include "include/roboteam_ai/analysis/PlaysObjects/Invariants/BallBelongsToUsInvariant.h"
#include "world_old/Ball.h"

namespace rtt::ai::analysis {

    bool BallBelongsToUsInvariant::isValid(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        auto margin = 0;
        auto ball = world->getBall();
        Vector2 ballPos = world->getBall()->getPos();

        bool ballNearGoalLine = ballPos.x < (field->get_field().get(LEFT_LINE).begin.x + margin);
        bool ballIsLayingStill = Vector2(ball->getVel()).length() < Constants::BALL_STILL_VEL();

        return ballNearGoalLine && ballIsLayingStill;
    }
}

