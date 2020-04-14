//
// Created by timovdk on 4/2/20.
//

#include "stp/invariants/BallMovesFastInvariant.h"

#include <include/roboteam_ai/stp/new_constants/ControlConstants.h>

namespace rtt::ai::stp::invariant {
double BallMovesFastInvariant::metricCheck(world_new::view::WorldDataView world, const Field* field) const noexcept {
    auto ballSpeed = world->getBall()->get()->getVelocity().length();
    return calculateMetric(ballSpeed - stp::control_constants::BALL_IS_MOVING_FAST_LIMIT);
}

double BallMovesFastInvariant::calculateMetric(const double& x) const noexcept {
    auto y = std::cos(x);
    return std::clamp(y, stp::control_constants::MIN_METRIC, stp::control_constants::MAX_METRIC);
}
}  // namespace rtt::ai::stp::invariant