//
// Created by jordi on 21-04-20.
/// Fuzzy Invariant based on the average distance of the CLOSEST ENEMY to EACH FRIENDLY
/// Range [0->DISTANCE_TO_ROBOT_CLOSE, DISTANCE_TO_ROBOT_FAR]
//

#include "stp/evaluations/global/FreedomOfRobotsGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {

FreedomOfRobotsGlobalEvaluation::FreedomOfRobotsGlobalEvaluation() noexcept {
    /**
     * Creates a piecewise linear function that looks as follows:
     *
     * (0,255)  |        XXXXXX
     *          |       XX
     *          |      XX
     *          |     XX
     *   (0,0)  |XXXXXX---------
     *              (Distance to closest enemy robot)
     */
    piecewiseLinearFunction = nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "freedomOfRobots");
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, 0.0);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, stp::control_constants::DISTANCE_TO_ROBOT_CLOSE + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_TRUE, stp::control_constants::DISTANCE_TO_ROBOT_FAR - stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, stp::control_constants::DISTANCE_TO_ROBOT_FAR - stp::control_constants::FUZZY_MARGIN);
}

uint8_t FreedomOfRobotsGlobalEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    auto& us = world->getWorld()->getUs();
    std::vector<uint8_t> distanceMetrics{};
    distanceMetrics.reserve(2 * us.size());

    // If there are no bots, ball is not close to us
    if (us.empty()) {
        return control_constants::FUZZY_FALSE;
    }

    for (auto robot : us) {
        auto robotPosition = robot.get()->getPos();
        auto distance{0.0};
        auto closestRobot = world->getWorld()->getRobotClosestToPoint(robotPosition, world::them);
        if (closestRobot.has_value() && closestRobot.value()) {
            distance = (closestRobot.value()->getPos() - robotPosition).length();
        }
        auto m = calculateMetric(distance);
        distanceMetrics.emplace_back(m);
    }

    if (distanceMetrics.empty()) {
        return control_constants::FUZZY_FALSE;
    }
    return std::accumulate(distanceMetrics.begin(), distanceMetrics.end(), 0) / distanceMetrics.size();
}

uint8_t FreedomOfRobotsGlobalEvaluation::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }

}  // namespace rtt::ai::stp::evaluation
