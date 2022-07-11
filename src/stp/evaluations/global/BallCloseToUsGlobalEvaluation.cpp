//
// Created by timovdk on 4/14/20.
/// Fuzzy Invariant how close the closest FRIENDLY is to the BALL
/// Range [0->BALL_IS_CLOSE, BALL_IS_CLOSE*2]
// TODO-Max check the distance values to be logical
//

#include "stp/evaluations/global/BallCloseToUsGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {

BallCloseToUsGlobalEvaluation::BallCloseToUsGlobalEvaluation() noexcept {
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
    piecewiseLinearFunction = nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "ballCloseToUs");
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, 0.0);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, stp::control_constants::BALL_IS_CLOSE + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_FALSE, stp::control_constants::BALL_IS_CLOSE * 2 - stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, stp::control_constants::BALL_IS_CLOSE * 2 - stp::control_constants::FUZZY_MARGIN);
}

uint8_t BallCloseToUsGlobalEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    auto& us = world->getWorld()->getUs();
    auto ballPos = world->getWorld()->getBall()->get()->position;
    std::vector<double> distances{};

    if (us.empty()) {
        RTT_ERROR("Us vector is empty")
        return control_constants::FUZZY_FALSE;
    }

    for (auto robot : us) {
        distances.emplace_back(robot.get()->getPos().dist(ballPos));
    }

    // If there are no distances, ball is not close to us
    if (distances.empty()) {
        return control_constants::FUZZY_FALSE;
    }
    return calculateMetric(*std::min_element(distances.begin(), distances.end()));
}

uint8_t BallCloseToUsGlobalEvaluation::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}  // namespace rtt::ai::stp::evaluation
