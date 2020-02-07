//
// Created by jessevw on 11.12.19.
//

#include "include/roboteam_ai/analysis/play-utilities/invariants/BallOnOurSideInvariant.h"

#include <roboteam_utils/Vector2.h>

#include "world/Ball.h"
#include "world/Field.h"
#include "world/World.h"

namespace rtt::ai::analysis {
using namespace rtt::ai::world;

bool BallOnOurSideInvariant::isValid(rtt::ai::world::World *world, const Field *field) {
    Vector2 ballPos = world->getBall()->getPos();

    // force the ball to be in the field
    bool inField = true;

    if (ballPos.x < 0) {
        return abs(ballPos.x) < field->getFieldLength() / 2 && abs(ballPos.y) < field->getFieldWidth() / 2;

        return true;
    }
    return false;
}
}  // namespace rtt::ai::analysis
