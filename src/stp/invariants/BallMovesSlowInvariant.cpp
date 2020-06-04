//
// Created by timovdk on 4/3/20.
//

#include "stp/invariants/BallMovesSlowInvariant.h"

namespace rtt::ai::stp::invariant {
BallMovesSlowInvariant::BallMovesSlowInvariant() noexcept {
    /**
     * Creates a piecewise linear function that looks as follows:
     *
     * (0,255)  |XXXXXX
     *          |     XX
     *          |      XX
     *          |       XX
     *   (0,0)  |--------XXXXXX
     *              (BallSpeed)
     */
    piecewiseLinearFunction = nativeformat::param::createParam(0, 255, 0, "testParam");
    piecewiseLinearFunction->setValueAtTime(255, 0.0);
    piecewiseLinearFunction->setValueAtTime(255, stp::control_constants::BALL_IS_MOVING_SLOW_LIMIT + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->linearRampToValueAtTime(0, stp::control_constants::BALL_GOT_SHOT_LIMIT - stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setValueAtTime(0, stp::control_constants::BALL_GOT_SHOT_LIMIT - stp::control_constants::FUZZY_MARGIN);
}

uint8_t BallMovesSlowInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept {
    auto ballSpeed = world->getBall()->get()->getVelocity().length();
    return calculateMetric(ballSpeed);
}

uint8_t BallMovesSlowInvariant::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->valueForTime(x); }
}  // namespace rtt::ai::stp::invariant