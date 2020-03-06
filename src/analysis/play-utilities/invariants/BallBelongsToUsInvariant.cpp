//
// Created by jessevw on 06.12.19.
//

#include "include/roboteam_ai/analysis/play-utilities/invariants/BallBelongsToUsInvariant.h"

#include <roboteam_utils/Vector2.h>
#include <include/roboteam_ai/world_new/views/WorldDataView.hpp>

#include "world/Ball.h"
#include "world/Field.h"
#include "world/World.h"

namespace rtt::ai::analysis {

bool BallBelongsToUsInvariant::isValid(world_new::view::WorldDataView world, const world::Field &field) {
    auto margin = 0;
    if (!world.getBall().has_value()) {
        return false;
    }
    auto ball = world.getBall().value();
    Vector2 ballPos = ball->getPos();

    bool ballNearGoalLine = ballPos.x < (field.getLeftmostX() + margin);
    bool ballIsLayingStill = Vector2(ball->getVelocity()).length() < Constants::BALL_STILL_VEL();

    return ballNearGoalLine && ballIsLayingStill;
}
}  // namespace rtt::ai::analysis
