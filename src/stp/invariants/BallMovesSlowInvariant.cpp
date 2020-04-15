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
    /**
     * If between FastLimit and FuzzyMargin -> slope
     * If < FuzzyMargin -> True
     * Else -> False
     *
     *     (255)
     *   XXXXXXX
     *       | XX
     *       |  XX
     *       |    XX
     *       |     XX
     *   ----+------XXXX
     *              (0)
     */
    uint8_t metric{};
    if (x >= stp::control_constants::FUZZY_MARGIN && x <= stp::control_constants::BALL_IS_MOVING_FAST_LIMIT) {
        metric = static_cast<uint8_t>(-182.1429 * x + 273.2143);
    } else if (x < stp::control_constants::FUZZY_MARGIN) {
        metric = stp::control_constants::FUZZY_TRUE;
    } else {
        metric = stp::control_constants::FUZZY_FALSE;
    }
    return metric;
}
}  // namespace rtt::ai::stp::invariant