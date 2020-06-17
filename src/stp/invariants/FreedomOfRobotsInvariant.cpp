//
// Created by jordi on 21-04-20.
//

#include "stp/invariants/FreedomOfRobotsInvariant.h"

namespace rtt::ai::stp::invariant {

FreedomOfRobotsInvariant::FreedomOfRobotsInvariant() noexcept {
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
    piecewiseLinearFunction = nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "distanceToClosestEnemyRobot");
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, 0.0);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, stp::control_constants::DISTANCE_TO_ROBOT_CLOSE + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_TRUE, stp::control_constants::DISTANCE_TO_ROBOT_FAR - stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, stp::control_constants::DISTANCE_TO_ROBOT_FAR - stp::control_constants::FUZZY_MARGIN);
}

uint8_t FreedomOfRobotsInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept {
    auto& us = world.getUs();
    std::vector<uint8_t> distanceMetrics{};
    distanceMetrics.reserve(2*us.size());

    for (auto robot : us) {
        auto robotPosition = robot.get()->getPos();
        auto distance{0.0};
        auto closestRobot = world.getRobotClosestToPoint(robotPosition, world_new::them);
        if(closestRobot.has_value() && closestRobot.value()) {
            distance = (closestRobot.value()->getPos() - robotPosition).length();
        }
        auto m = calculateMetric(distance);
        distanceMetrics.emplace_back(m);
    }

    return std::accumulate(distanceMetrics.begin(), distanceMetrics.end(), 0) / distanceMetrics.size();
}

uint8_t FreedomOfRobotsInvariant::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }

}  // namespace rtt::ai::stp::invariant
