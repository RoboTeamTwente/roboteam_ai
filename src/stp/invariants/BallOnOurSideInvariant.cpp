//
// Created by timovdk on 4/14/20.
//

#include "stp/invariants/BallOnOurSideInvariant.h"
#include <utilities/GameStateManager.hpp>

namespace rtt::ai::stp::invariant {
bool BallOnOurSideInvariant::checkInvariant(world_new::view::WorldDataView world, const Field *field) const noexcept {
    // If our side is left, x of the ball is <= 0 when the ball is on our side
    // If our side is right, x of the ball is >= 0 when the ball is on our side
    return SETTINGS.isLeft() ? world.getBall().value()->getPos().x <= 0 : world.getBall().value()->getPos().x >= 0;
}
}