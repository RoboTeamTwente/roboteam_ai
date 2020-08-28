//
// Created by timovdk on 4/14/20.
//

#include "stp/invariants/BallOnOurSideInvariant.h"

namespace rtt::ai::stp::invariant {
uint8_t BallOnOurSideInvariant::metricCheck(world::view::WorldDataView world, const world::Field *field) const noexcept {
    return world.getBall().value()->getPos().x < 0 ? control_constants::FUZZY_TRUE : control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::invariant