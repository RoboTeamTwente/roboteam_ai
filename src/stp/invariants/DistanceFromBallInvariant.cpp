//
// Created by timovdk on 4/21/20.
//

#include "stp/invariants/DistanceFromBallInvariant.h"

namespace rtt::ai::stp::invariant {
DistanceFromBallInvariant::DistanceFromBallInvariant() noexcept {
    /**
     * Creates a piecewise linear function that looks as follows:
     *
     * (0,255)  |XX
     *          | XXXXXX
     *          |      XXXXXX
     *          |           XXXXXX
     *   (0,0)  |----------------XXXXXXX
     *              (Distance to Ball)
     */
    piecewiseLinearFunction = nativeformat::param::createParam(0, 255, 0, "testParam");
    piecewiseLinearFunction->setYAtX(255, 0.0);
    piecewiseLinearFunction->linearRampToYAtX(0, stp::control_constants::BALL_IS_CLOSE * 4 + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setYAtX(0, stp::control_constants::BALL_IS_CLOSE * 4 + stp::control_constants::FUZZY_MARGIN);
}

uint8_t DistanceFromBallInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept {
    auto& us = world.getUs();
    auto ballPos = world.getBall()->get()->getPos();
    std::vector<double> distanceMetrics{};
    distanceMetrics.reserve(control_constants::MAX_ROBOT_COUNT);

    std::transform(us.begin(), us.end(), std::back_inserter(distanceMetrics), [&](auto& robot) { return robot.get()->getPos().dist(ballPos); });

    return calculateMetric(*std::min_element(distanceMetrics.begin(), distanceMetrics.end()));
}

uint8_t DistanceFromBallInvariant::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}  // namespace rtt::ai::stp::invariant