//
// Created by Alexander on 28-01-2022
//

#include "stp/evaluations/game_states/KickOffUsOrNormalGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t KickOffUsOrNormalGameStateEvaluation::metricCheck(const world::World *, const world::Field *) const noexcept {
    return (GameStateManager::getCurrentGameState().getStrategyName() == "kickoff_us" || GameStateManager::getCurrentGameState().getStrategyName() == "normal_play")
               ? stp::control_constants::FUZZY_TRUE
               : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation