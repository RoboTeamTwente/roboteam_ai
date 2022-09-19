//
// Created by jordi on 12-05-20.
/// Fuzzy Invariant based on the openness of the ENEMY GOAL from BALL
/// Range [0% , 100%]
// TODO-Max specify US or THEM
//

#include "stp/evaluations/global/GoalVisionFromBallGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {

GoalVisionFromBallGlobalEvaluation::GoalVisionFromBallGlobalEvaluation() noexcept {
    /**
     * Creates a piecewise linear function that looks as follows:
     *
     * (0,255)  |            XXXX
     *          |         XXXX
     *          |      XXXX
     *          |   XXXX
     *   (0,0)  |XXXX---------
     *              (visibility in %)
     *
     * For 0% visibility return 0, for 100% visibility return 255
     */
    piecewiseLinearFunction = nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "goalVisionFromBall");
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, 0.0);
    piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_TRUE, 100);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, 100);
}

uint8_t GoalVisionFromBallGlobalEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    return calculateMetric(
        FieldComputations::getPercentageOfGoalVisibleFromPoint(*field, false, world->getWorld()->getBall().value()->position, world->getWorld().value(), -1, true));
}

uint8_t GoalVisionFromBallGlobalEvaluation::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }

}  // namespace rtt::ai::stp::evaluation