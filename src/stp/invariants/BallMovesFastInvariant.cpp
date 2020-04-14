//
// Created by timovdk on 4/2/20.
//

#include "stp/invariants/BallMovesFastInvariant.h"

namespace rtt::ai::stp::invariant {
bool BallMovesFastInvariant::checkInvariant(world_new::view::WorldDataView world, const Field *field) const noexcept {
    auto ballSpeed = world->getBall()->get()->getVelocity().length();
    return ballSpeed > stp::control_constants::BALL_IS_MOVING_FAST_LIMIT;
}
}  // namespace rtt::ai::stp::invariant