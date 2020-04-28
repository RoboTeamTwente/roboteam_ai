//
// Created by jordi on 28-04-20.
//

#include "stp/invariants/BallIsFreeInvariant.h"

namespace rtt::ai::stp::invariant {

uint8_t BallIsFreeInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field *field) const noexcept {
    auto& robots = world.getRobots();
    return std::any_of(robots.begin(), robots.end(), [](auto& robot) { return robot.hasBall(); }) ? stp::control_constants::FUZZY_FALSE : stp::control_constants::FUZZY_TRUE;
}

}  // namespace rtt::ai::stp::invariant