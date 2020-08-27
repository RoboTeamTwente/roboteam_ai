//
// Created by jordi on 28-04-20.
//

#include "stp/invariants/BallIsFreeInvariant.h"

namespace rtt::ai::stp::invariant {

uint8_t BallIsFreeInvariant::metricCheck(world::view::WorldDataView world, const world::Field* field) const noexcept {
    auto& robots = world.getRobotsNonOwning();

    // If there are no robots, ball is free
    if (robots.empty()) {
        return stp::control_constants::FUZZY_TRUE;
    }
    return std::any_of(robots.begin(), robots.end(), [](auto& robot) { return robot.hasBall(); }) ? stp::control_constants::FUZZY_FALSE : stp::control_constants::FUZZY_TRUE;
}
}  // namespace rtt::ai::stp::invariant