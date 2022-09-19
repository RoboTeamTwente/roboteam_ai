//
// Created by jaro on 30-10-20.
/// T/F Invariant if the ball is on ENEMY side
// TODO-Max make this fuzzy based on how far from middle the ball is
//

#include "stp/evaluations/global/BallOnTheirSideGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t BallOnTheirSideGlobalEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    return world->getWorld()->getBall().value()->position.x >= 0 ? control_constants::FUZZY_TRUE : control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation