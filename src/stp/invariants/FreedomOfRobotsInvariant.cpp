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
    piecewiseLinearFunction->setValueAtTime(control_constants::FUZZY_FALSE, 0.0);
    piecewiseLinearFunction->setValueAtTime(control_constants::FUZZY_FALSE, stp::control_constants::DISTANCE_TO_ROBOT_CLOSE + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->linearRampToValueAtTime(control_constants::FUZZY_TRUE, stp::control_constants::DISTANCE_TO_ROBOT_FAR - stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setValueAtTime(control_constants::FUZZY_TRUE, stp::control_constants::DISTANCE_TO_ROBOT_FAR - stp::control_constants::FUZZY_MARGIN);
}

uint8_t FreedomOfRobotsInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept {
    auto& us = world.getUs();
    std::vector<uint8_t> distanceMetrics{};

    std::transform(us.begin(), us.end(), std::back_inserter(distanceMetrics), [&](auto& robot) {
        auto robotPosition = robot.get()->getPos();
        auto distance{0.0};
        if(world.getRobotClosestToPoint(robotPosition, world_new::them)) distance = (world.getRobotClosestToPoint(robotPosition, world_new::them).value()->getPos() - robotPosition).length();
        return calculateMetric(distance);
    });

    return std::accumulate(distanceMetrics.begin(), distanceMetrics.end(), 0) / distanceMetrics.size();
}

uint8_t FreedomOfRobotsInvariant::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->valueForTime(x); }

}  // namespace rtt::ai::stp::invariant