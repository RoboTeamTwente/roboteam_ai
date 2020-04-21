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
    piecewiseLinearFunction = nativeformat::param::createParam(0, 255, 0, "testParam");
    piecewiseLinearFunction->setYAtX(255, 0.0);
    piecewiseLinearFunction->setYAtX(255, stp::control_constants::BALL_IS_CLOSE + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->linearRampToYAtX(0, stp::control_constants::BALL_IS_CLOSE * 2 - stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setYAtX(0, stp::control_constants::BALL_IS_CLOSE * 2 - stp::control_constants::FUZZY_MARGIN);
}

uint8_t BallCloseToUsInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept {
    auto& us = world.getUs();
    auto ballPos = world.getBall()->get()->getPos();
    std::vector<uint8_t> distanceMetrics{};

    std::transform(us.begin(), us.end(), std::back_inserter(distanceMetrics), [&](auto& robot) { return calculateMetric(robot.get()->getPos().dist(ballPos)); });

    return *std::max_element(distanceMetrics.begin(), distanceMetrics.end());
}

uint8_t BallCloseToUsInvariant::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}  // namespace rtt::ai::stp::invariant