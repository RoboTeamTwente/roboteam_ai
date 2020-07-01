//
// Created by ratoone on 27-03-20.
//

#include "stp/invariants/WeHaveBallInvariant.h"

namespace rtt::ai::stp::invariant {
uint8_t WeHaveBallInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field *field) const noexcept {
    auto& us = world.getUs();

    // If there are no bots, we don't have ball
    if(us.empty()) {
        return stp::control_constants::FUZZY_FALSE;
    }
    return std::any_of(us.begin(), us.end(), [](auto& robot) { return robot.hasBall(); }) ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::invariant
