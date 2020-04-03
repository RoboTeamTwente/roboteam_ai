//
// Created by timo on 4/3/20.
//

#include "stp/invariants/BallMovesSlowInvariant.h"

namespace rtt::ai::stp::invariant {
bool BallMovesSlowInvariant::checkInvariant(world_new::view::WorldDataView world, const Field *field) const noexcept {
    if (world->getBall().has_value())
        return world->getBall()->get()->getVelocity().length() < stp::control_constants::BALL_IS_MOVING_VEL * 3;

    else
        return false;
}
}  // namespace rtt::ai::stp::invariant