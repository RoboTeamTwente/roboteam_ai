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
    auto y = std::cos(x);
    return std::clamp(y, stp::control_constants::MIN_METRIC, stp::control_constants::MAX_METRIC);
}
}  // namespace rtt::ai::stp::invariant