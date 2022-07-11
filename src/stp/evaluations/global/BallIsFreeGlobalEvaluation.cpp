//
// Created by jordi on 28-04-20.
/// T/F Invariant if a ROBOT has the BALL
// TODO check if this works, else change to distance to ball limit
//

#include "stp/evaluations/global/BallIsFreeGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {

uint8_t BallIsFreeGlobalEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    auto& robots = world->getWorld()->getRobotsNonOwning();

    // If there are no robots, ball is free
    if (robots.empty()) {
        return stp::control_constants::FUZZY_TRUE;
    }
    return std::any_of(robots.begin(), robots.end(), [](auto& robot) { return robot->hasBall(); }) ? stp::control_constants::FUZZY_FALSE : stp::control_constants::FUZZY_TRUE;
}
}  // namespace rtt::ai::stp::evaluation