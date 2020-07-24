//
// Created by jordi on 06-05-20.
//

#include "stp/invariants/BallCloseToThemInvariant.h"

namespace rtt::ai::stp::invariant {

BallCloseToThemInvariant::BallCloseToThemInvariant() noexcept {
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
    piecewiseLinearFunction = nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "ballCloseToThem");
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, 0.0);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, stp::control_constants::BALL_IS_CLOSE + stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_FALSE, stp::control_constants::BALL_IS_CLOSE * 2 - stp::control_constants::FUZZY_MARGIN);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, stp::control_constants::BALL_IS_CLOSE * 2 - stp::control_constants::FUZZY_MARGIN);
}

uint8_t BallCloseToThemInvariant::metricCheck(world::view::WorldDataView world, const world::Field* field) const noexcept {
    auto robot = world.getRobotClosestToBall(world::them);
    if(robot) return calculateMetric(robot.value()->getDistanceToBall());
    else return 0;
}

uint8_t BallCloseToThemInvariant::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }

}  // namespace rtt::ai::stp::invariant
