//
// Created by roboteam on 23/6/20.
/// T/F Invariant if FRIENDLY is closest to BALL
//

#include "stp/invariants/BallClosestToUsInvariant.h"

namespace rtt::ai::stp::invariant {

uint8_t BallClosestToUsInvariant::metricCheck(world::view::WorldDataView world, const world::Field* field) const noexcept {
    auto closestBall = world.getRobotClosestToBall();
    if (closestBall && closestBall->get()->getTeam() == world::us) {
        return control_constants::FUZZY_TRUE;
    } else
        return control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::invariant
