//
// Created by timo on 4/3/20.
//

#include "stp/invariants/BallMovesSlowInvariant.h"

namespace rtt::ai::stp::invariant {
bool BallMovesSlowInvariant::checkInvariant(world_new::view::WorldDataView world, const Field *field) const noexcept {
    auto ballSpeed = world->getBall()->get()->getVelocity().length();
    return ballSpeed < stp::control_constants::BALL_IS_MOVING_FAST_LIMIT;
}
}  // namespace rtt::ai::stp::invariant