//
// Created by timovdk on 4/3/20.
//

#include "stp/invariants/BallMovesSlowInvariant.h"

namespace rtt::ai::stp::invariant {
uint8_t BallMovesSlowInvariant::metricCheck(world_new::view::WorldDataView world, const Field* field) const noexcept {
    auto ballSpeed = world->getBall()->get()->getVelocity().length();
    return calculateMetric(ballSpeed - stp::control_constants::BALL_IS_MOVING_SLOW_LIMIT);
}

uint8_t BallMovesSlowInvariant::calculateMetric(const double& x) const noexcept {
    uint8_t metric{};
    // If the difference is between -ball_is_moving_slow_limit and 0 return 255
    // Else if the difference is between 0 and 1, make a slope to fuzz the boolean
    // Else return 0, since this is too fast to classify as slow
    if (x >= -stp::control_constants::BALL_IS_MOVING_SLOW_LIMIT && x <= 0) {
        metric = 255;
    } else if (x >= 0 && x <= 1) {
        metric = static_cast<uint8_t>(-255 * x + 255);
    } else {
        metric = 0;
    }
    return metric;
}
}  // namespace rtt::ai::stp::invariant