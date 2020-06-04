//
// Created by timovdk on 5/27/20.
//

#include "stp/invariants/NoGoalVisionFromBallInvariant.h"

namespace rtt::ai::stp::invariant {

NoGoalVisionFromBallInvariant::NoGoalVisionFromBallInvariant() noexcept {
    piecewiseLinearFunction = nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "NoGoalVisionFromBall");
    piecewiseLinearFunction->setValueAtTime(control_constants::FUZZY_FALSE, 100);
    piecewiseLinearFunction->linearRampToValueAtTime(control_constants::FUZZY_TRUE, 0.0);
    piecewiseLinearFunction->setValueAtTime(control_constants::FUZZY_TRUE, 0.0);
}

uint8_t NoGoalVisionFromBallInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept {
    return calculateMetric(FieldComputations::getPercentageOfGoalVisibleFromPoint(*field, false, world.getBall().value()->getPos(), world, -1, false));
}

uint8_t NoGoalVisionFromBallInvariant::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->valueForTime(x); }

}  // namespace rtt::ai::stp::invariant
