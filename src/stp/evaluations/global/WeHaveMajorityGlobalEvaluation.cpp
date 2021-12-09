//
// Created by luukkn on 21-04-20.
/// T/F Invariant if size of TEAM FRIENDLY > TEAM ENEMY
//

#include "stp/evaluations/global/WeHaveMajorityGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t WeHaveMajorityGlobalEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    return world->getWorld()->getUs().size() > world->getWorld()->getThem().size() ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation