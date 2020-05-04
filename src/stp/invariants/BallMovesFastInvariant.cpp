//
// Created by timo on 4/2/20.
//

#include "stp/invariants/BallMovesFastInvariant.h"

namespace rtt::ai::stp::invariant {
bool BallMovesFastInvariant::checkInvariant(world_new::view::WorldDataView world, const world::Field *field) const noexcept {
    if (world->getBall().has_value()) {
        auto ballSpeed = world->getBall()->get()->getVelocity().length();
        return ballSpeed > stp::control_constants::BALL_IS_MOVING_FAST_LIMIT;
    }
    return false;
}
}  // namespace rtt::ai::stp::invariant