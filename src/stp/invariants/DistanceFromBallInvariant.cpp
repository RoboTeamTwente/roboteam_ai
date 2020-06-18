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
    piecewiseLinearFunction = nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "distanceFromBall");
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, 0.0);
    piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_FALSE, stp::control_constants::BALL_IS_CLOSE * 4 + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, stp::control_constants::BALL_IS_CLOSE * 4 + stp::control_constants::FUZZY_MARGIN);
}

uint8_t DistanceFromBallInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept {
    auto& us = world.getUs();
    auto ballPos = world.getBall()->get()->getPos();
    std::vector<double> distances{};
    distances.reserve(control_constants::MAX_ROBOT_COUNT);

    // If there are no bots, ball is not close to us
    if(us.empty()) {
        return control_constants::FUZZY_FALSE;
    }
    std::transform(us.begin(), us.end(), std::back_inserter(distances), [&](auto& robot) { return robot.get()->getPos().dist(ballPos); });

    return calculateMetric(*std::min_element(distances.begin(), distances.end()));
}

uint8_t DistanceFromBallInvariant::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}  // namespace rtt::ai::stp::invariant
