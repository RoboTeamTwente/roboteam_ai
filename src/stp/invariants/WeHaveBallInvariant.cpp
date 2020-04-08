//
// Created by ratoone on 27-03-20.
//

#include "stp/invariants/WeHaveBallInvariant.h"

namespace rtt::ai::stp::invariant {
    bool WeHaveBallInvariant::checkInvariant(world_new::view::WorldDataView world, const Field *field) const noexcept {
        auto& us = world.getUs();
        return std::any_of(us.begin(), us.end(), [](auto& robot){return robot.hasBall();});
    }
}
