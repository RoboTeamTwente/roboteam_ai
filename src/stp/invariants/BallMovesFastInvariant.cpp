//
// Created by timovdk on 4/2/20.
//

#include "stp/invariants/BallMovesFastInvariant.h"

#include <stp/new_constants/ControlConstants.h>

namespace rtt::ai::stp::invariant {
uint8_t BallMovesFastInvariant::metricCheck(world_new::view::WorldDataView world, const Field* field) const noexcept {
    auto ballSpeed = world->getBall()->get()->getVelocity().length();
    return calculateMetric(ballSpeed - stp::control_constants::BALL_IS_MOVING_FAST_LIMIT);
}

uint8_t BallMovesFastInvariant::calculateMetric(const double& x) const noexcept {
    uint8_t metric{};
    // If the difference is between -1 and 0, make a slope to fuzz the boolean evaluation
    // Else if the difference is 0 or higher return 255
    // Else return 0, since this is too slow to classify as moving fast
    if (x >= -1 && x <= 0) {
        metric = static_cast<uint8_t>(255 * x + 255);
    } else if (x >= 0) {
        metric = 255;
    } else {
        metric = 0;
    }
    return metric;
}
}  // namespace rtt::ai::stp::invariant