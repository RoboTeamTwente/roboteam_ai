//
// Created by jaro on 30-10-20.
//

#include "include/roboteam_ai/stp/invariants/BallOnTheirSideInvariant.h"

namespace rtt::ai::stp::invariant {
    uint8_t BallOnTheirSideInvariant::metricCheck(world::view::WorldDataView world, const world::Field *field) const noexcept {
        return world.getBall().value()->getPos().x >= 0 ? control_constants::FUZZY_TRUE : control_constants::FUZZY_FALSE;
    }
}  // namespace rtt::ai::stp::invariant