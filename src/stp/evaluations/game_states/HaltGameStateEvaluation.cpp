//
// Created by ratoone on 27-03-20.
//

#include "stp/evaluations/game_states/HaltGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {
uint8_t HaltGameStateEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    return GameStateManager::getCurrentGameState().getStrategyName() == "halt" ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
