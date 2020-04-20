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
    piecewiseLinearFunction->setYAtX(255, 0.0);
    piecewiseLinearFunction->setYAtX(255, stp::control_constants::BALL_IS_MOVING_SLOW_LIMIT + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->linearRampToYAtX(0, stp::control_constants::BALL_GOT_SHOT_LIMIT - stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setYAtX(0, stp::control_constants::BALL_GOT_SHOT_LIMIT - stp::control_constants::FUZZY_MARGIN);
}

uint8_t BallMovesSlowInvariant::metricCheck(world_new::view::WorldDataView world, const Field* field) const noexcept {
    auto ballSpeed = world->getBall()->get()->getVelocity().length();
    return calculateMetric(ballSpeed);
}

uint8_t BallMovesSlowInvariant::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}  // namespace rtt::ai::stp::invariant