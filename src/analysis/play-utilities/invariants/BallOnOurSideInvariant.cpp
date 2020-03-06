//
// Created by jessevw on 11.12.19.
//

#include "include/roboteam_ai/analysis/play-utilities/invariants/BallOnOurSideInvariant.h"
#include <roboteam_utils/Vector2.h>
#include <include/roboteam_ai/world_new/views/WorldDataView.hpp>

namespace rtt::ai::analysis {
using namespace rtt::ai::world;

bool BallOnOurSideInvariant::isValid(world_new::view::WorldDataView world, const Field &field) {
    auto ballOpt = world.getBall();
    if (!ballOpt.has_value()) return false;

    auto ball = ballOpt.value();
    Vector2 ballPos = ball->getPos();
    if (ballPos.x < 0) {
        return abs(ballPos.x) < field.getFieldLength() / 2 && abs(ballPos.y) < field.getFieldWidth() / 2;
    }
    return false;
}
}  // namespace rtt::ai::analysis
