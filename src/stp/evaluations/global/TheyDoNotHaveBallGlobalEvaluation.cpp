//
// Created by Alexander on 27-03-20.
/// T/F Invariant if ENEMY does not have BALL
//

#include "stp/evaluations/global/TheyDoNotHaveBallGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {
uint8_t TheyDoNotHaveBallGlobalEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    auto& them = world->getWorld()->getThem();

    // If there are no bots, they don't have ball
    if (them.empty()) {
        return stp::control_constants::FUZZY_TRUE;
    }

    // If we have the ball, return true, as the enemy will either not have the ball, or we both have it (in which case, we usually want to act as if they don't have it)
    auto& us  = world->getWorld()->getUs();
    if (std::any_of(us.begin(), us.end(), [](auto& bot) { return bot.hasBall(); })) return stp::control_constants::FUZZY_TRUE;

    return std::any_of(them.begin(), them.end(), [](auto& robot) { return robot.hasBall(); }) ? stp::control_constants::FUZZY_FALSE : stp::control_constants::FUZZY_TRUE;
}
}  // namespace rtt::ai::stp::evaluation
