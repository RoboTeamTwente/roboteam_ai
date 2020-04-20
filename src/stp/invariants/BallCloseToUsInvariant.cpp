//
// Created by timovdk on 4/14/20.
//

#include "stp/invariants/BallCloseToUsInvariant.h"

namespace rtt::ai::stp::invariant {
uint8_t BallCloseToUsInvariant::metricCheck(world_new::view::WorldDataView world, const Field* field) const noexcept {
    auto& us = world.getUs();
    auto ballPos = world.getBall()->get()->getPos();
    // If the distance between ballpos and robotpos is smaller than BALL_IS_CLOSE constant return true
    return std::any_of(us.begin(), us.end(), [&ballPos](auto& robot) { return robot.get()->getPos().dist(ballPos) < control_constants::BALL_IS_CLOSE; })
               ? control_constants::FUZZY_TRUE
               : control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::invariant