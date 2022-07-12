//
// Created by alexander on 13-07-22.
//

#include "stp/evaluations/game_states/WeHaveFreeKickAfterEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t WeHaveFreeKickAfterEvaluation::metricCheck(const world::World *, const world::Field *) const noexcept {
    return (GameStateManager::getNextRefCommand() == RefCommand::INDIRECT_FREE_US || GameStateManager::getNextRefCommand() == RefCommand::DIRECT_FREE_US) ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation