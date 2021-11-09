//
// Created by jordi on 28-04-20.
//

#include "stp/evaluations/game_states/PenaltyUsPrepareGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t PenaltyUsPrepareGameStateEvaluation::metricCheck(const world::World *, const world::Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getStrategyName() == "penalty_us_prepare" ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation