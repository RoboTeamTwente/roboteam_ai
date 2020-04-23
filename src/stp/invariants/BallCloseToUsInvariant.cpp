//
// Created by timovdk on 4/14/20.
//

#include "stp/invariants/BallCloseToUsInvariant.h"

namespace rtt::ai::stp::invariant {

BallCloseToUsInvariant::BallCloseToUsInvariant() noexcept {
    /**
     * Creates a piecewise linear function that looks as follows:
     *
     * (0,255)  |XXXXXX
     *          |     XX
     *          |      XX
     *          |       XX
     *   (0,0)  |--------XXXXXX
     *              (Distance from the ball)
     */
    piecewiseLinearFunction = nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "testParam");
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, 0.0);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, stp::control_constants::BALL_IS_CLOSE + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_FALSE, stp::control_constants::BALL_IS_CLOSE * 2 - stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, stp::control_constants::BALL_IS_CLOSE * 2 - stp::control_constants::FUZZY_MARGIN);
}

uint8_t BallCloseToUsInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept {
    auto& us = world.getUs();
    auto ballPos = world.getBall()->get()->getPos();
    std::vector<double> distances{};

    std::transform(us.begin(), us.end(), std::back_inserter(distances), [&](auto& robot) { return robot.get()->getPos().dist(ballPos); });

    return calculateMetric(*std::min_element(distances.begin(), distances.end()));
}

uint8_t BallCloseToUsInvariant::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}  // namespace rtt::ai::stp::invariant
