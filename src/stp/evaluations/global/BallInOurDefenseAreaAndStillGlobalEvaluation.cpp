//
// Created by Alexander on 29-01-2022
/// T/F Invariant if the ball is in our defense area and not moving
//

#include "stp/evaluations/global/BallInOurDefenseAreaAndStillGlobalEvaluation.h"

#include "stp/constants/ControlConstants.h"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::evaluation {
uint8_t BallInOurDefenseAreaAndStillGlobalEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    return (FieldComputations::pointIsInOurDefenseArea(*field, world->getWorld()->getBall()->get()->position) &&
            world->getWorld()->getBall()->get()->velocity.length() < control_constants::BALL_STILL_VEL)
               ? control_constants::FUZZY_TRUE
               : control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation