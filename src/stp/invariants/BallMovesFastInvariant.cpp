//
// Created by timovdk on 4/2/20.
//

#include "stp/invariants/BallMovesFastInvariant.h"

#include <stp/new_constants/ControlConstants.h>

namespace rtt::ai::stp::invariant {
BallMovesFastInvariant::BallMovesFastInvariant() noexcept {
    piecewiseLinearFunction = nativeformat::param::createParam(0, 255, 0, "ballMovesFast");
    piecewiseLinearFunction->setValueAtTime(0.0, 0.0);
    piecewiseLinearFunction->setValueAtTime(0.0, stp::control_constants::BALL_IS_MOVING_SLOW_LIMIT + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->linearRampToValueAtTime(255, stp::control_constants::BALL_IS_MOVING_FAST_LIMIT - stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setValueAtTime(255, stp::control_constants::BALL_IS_MOVING_FAST_LIMIT - stp::control_constants::FUZZY_MARGIN);
}

uint8_t BallMovesFastInvariant::metricCheck(world_new::view::WorldDataView world, const Field* field) const noexcept {
    auto ballSpeed = world->getBall()->get()->getVelocity().length();
    return calculateMetric(ballSpeed);
}

/**
 *         (255)
 * |       XXXXXXX
 * |      XX
 * |     XX
 * |    XX
 * |   XX
 * |XXXX----------
 * (0)
 */
uint8_t BallMovesFastInvariant::calculateMetric(const double& x) const noexcept {
    return piecewiseLinearFunction->valueForTime(x);
}
}  // namespace rtt::ai::stp::invariant