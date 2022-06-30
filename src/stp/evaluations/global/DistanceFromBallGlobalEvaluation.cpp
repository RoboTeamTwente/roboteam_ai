//
// Created by timovdk on 4/21/20.
/// Fuzzy Invariant how close the closest FRIENDLY is to the BALL
/// Range [0, BALL_IS_CLOSE*4]
// TODO-Max check the distance values to be logical
//

#include "stp/evaluations/global/DistanceFromBallGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
DistanceFromBallGlobalEvaluation::DistanceFromBallGlobalEvaluation() noexcept {
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
    piecewiseLinearFunction =
        nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "distanceFromBallGlobalEvaluation");
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, 0.0);
    piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_FALSE, stp::control_constants::BALL_IS_CLOSE * 4 + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, stp::control_constants::BALL_IS_CLOSE * 4 + stp::control_constants::FUZZY_MARGIN);
}

uint8_t DistanceFromBallGlobalEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    auto& us = world->getWorld()->getUs();
    auto ballPos = world->getWorld()->getBall()->get()->position;
    std::vector<double> distances{};
    distances.reserve(control_constants::MAX_ROBOT_COUNT);

    // If there are no bots, ball is not close to us
    if (us.empty()) {
        return control_constants::FUZZY_FALSE;
    }
    std::transform(us.begin(), us.end(), std::back_inserter(distances), [&](auto& robot) { return robot.get()->getPos().dist(ballPos); });

    return calculateMetric(*std::min_element(distances.begin(), distances.end()));
}

uint8_t DistanceFromBallGlobalEvaluation::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}  // namespace rtt::ai::stp::evaluation
