//
// Created by jessevw on 06.12.19.
//

#include "stp/play-utilities/invariants/BallBelongsToUsInvariant.h"

#include <roboteam_utils/Vector2.h>

#include "world/Ball.h"
#include "world/Field.h"
#include "world/World.h"

namespace rtt::ai::analysis {
using namespace rtt::ai::world;

bool BallBelongsToUsInvariant::isValid(rtt::ai::world::World *world, const Field &field) {
    auto margin = 0;
    auto ball = world->getBall();
    Vector2 ballPos = world->getBall()->getPos();

    bool ballNearGoalLine = ballPos.x < (field.getLeftmostX() + margin);
    bool ballIsLayingStill = Vector2(ball->getVel()).length() < Constants::BALL_STILL_VEL();

    return ballNearGoalLine && ballIsLayingStill;
}
}  // namespace rtt::ai::analysis
