//
// Created by ratoone on 15-05-20.
//

#include "stp/evaluations/game_states/NormalOrFreeKickUsGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t NormalOrFreeKickUsGameStateEvaluation::metricCheck(const world::World *, const world::Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getStrategyName() == "free_kick_us" || GameStateManager::getCurrentGameState().getStrategyName() == "normal_play"
               ? stp::control_constants::FUZZY_TRUE
               : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation