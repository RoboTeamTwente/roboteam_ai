//
// Created by ratoone on 27-03-20.
/// T/F Invariant if ENEMY has BALL
//

#include "stp/evaluations/global/TheyHaveBallGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t TheyHaveBallGlobalEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    auto& them = world->getWorld()->getThem();

    // If there are no bots, they don't have ball
    if (them.empty()) {
        return stp::control_constants::FUZZY_FALSE;
    }
    return std::any_of(them.begin(), them.end(), [](auto& robot) { return robot->hasBall(); }) ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
