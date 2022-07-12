//
// Created by alexander on 13-07-22.
//

#include "stp/evaluations/game_states/TheyHaveFreeKickAfterEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t TheyHaveFreeKickAfterEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    return (GameStateManager::getNextRefCommand() == RefCommand::INDIRECT_FREE_THEM || GameStateManager::getNextRefCommand() == RefCommand::DIRECT_FREE_THEM) ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation