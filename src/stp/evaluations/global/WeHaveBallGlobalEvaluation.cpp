//
// Created by ratoone on 27-03-20.
/// T/F Invariant if FRIENDLY has BALL
//

#include "stp/evaluations/global/WeHaveBallGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t WeHaveBallGlobalEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    auto& us = world->getWorld()->getUs();

    // If there are no bots, we don't have ball
    if (us.empty()) {
        return stp::control_constants::FUZZY_FALSE;
    }
    return std::any_of(us.begin(), us.end(), [](auto& robot) { return robot->hasBall(); }) ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
