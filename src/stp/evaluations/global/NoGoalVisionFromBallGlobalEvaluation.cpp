//
// Created by timovdk on 5/27/20.
/// Fuzzy Invariant based on the openness of the ENEMY GOAL from BALL
/// Range [100% , 0%]
// TODO-Max check usability, as invertion of GoalVisionFromBall
//

#include "stp/evaluations/global/NoGoalVisionFromBallGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {

NoGoalVisionFromBallGlobalEvaluation::NoGoalVisionFromBallGlobalEvaluation() noexcept {
    piecewiseLinearFunction =
        nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "NoGoalVisionFromBall");
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, 100);
    piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_TRUE, 0.0);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, 0.0);
}

uint8_t NoGoalVisionFromBallGlobalEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    return calculateMetric(
        FieldComputations::getPercentageOfGoalVisibleFromPoint(*field, false, world->getWorld()->getBall().value()->position, world->getWorld().value(), -1, false));
}

uint8_t NoGoalVisionFromBallGlobalEvaluation::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }

}  // namespace rtt::ai::stp::evaluation
