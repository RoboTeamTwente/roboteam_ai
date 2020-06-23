//
// Created by roboteam on 23/6/20.
//

#include "include/roboteam_ai/stp/invariants/BallClosestToUsInvariant.h"

namespace rtt::ai::stp::invariant {


uint8_t BallClosestToUsInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept {
    auto closestDistUs = world.getRobotClosestToBall();
    if (closestDistUs->get()->getTeam() == world_new::us) {
        return control_constants::FUZZY_TRUE;
    }
    else return control_constants::FUZZY_FALSE;

}
BallClosestToUsInvariant::BallClosestToUsInvariant() noexcept {

}

}  // namespace rtt::ai::stp::invariant
