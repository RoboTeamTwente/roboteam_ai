//
// Created by timovdk on 4/2/20.
/// Fuzzy Invariant based on the current speed of the ball
/// Range [0->BALL_IS_MOVING_SLOW_LIMIT, BALL_GOT_SHOT_LIMIT]
//

#include "stp/evaluations/global/BallGotShotGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
BallGotShotGlobalEvaluation::BallGotShotGlobalEvaluation() noexcept {
    /**
     * Creates a piecewise linear function that looks as follows:
     *
     * (0,255)  |        XXXXXX
     *          |       XX
     *          |      XX
     *          |     XX
     *   (0,0)  |XXXXXX---------
     *              (BallSpeed)
     */
    piecewiseLinearFunction = nativeformat::param::createParam(0, 255, 0, "ballGotShot");
    piecewiseLinearFunction->setYAtX(0.0, 0.0);
    piecewiseLinearFunction->setYAtX(0.0, stp::control_constants::BALL_IS_MOVING_SLOW_LIMIT + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->linearRampToYAtX(255, stp::control_constants::BALL_GOT_SHOT_LIMIT - stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setYAtX(255, stp::control_constants::BALL_GOT_SHOT_LIMIT - stp::control_constants::FUZZY_MARGIN);
}

uint8_t BallGotShotGlobalEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    return calculateMetric(world->getWorld()->getBall()->get()->velocity.length());
}

uint8_t BallGotShotGlobalEvaluation::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}  // namespace rtt::ai::stp::evaluation