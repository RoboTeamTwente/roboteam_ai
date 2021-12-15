//
// Created by jordi on 28-04-20.
//

#include "stp/evaluations/game_states/NormalPlayGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t NormalPlayGameStateEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    return GameStateManager::getCurrentGameState().getStrategyName() == "normal_play" ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation