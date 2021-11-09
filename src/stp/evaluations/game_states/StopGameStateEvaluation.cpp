//
// Created by jordi on 28-04-20.
//

#include "stp/evaluations/game_states/StopGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {

uint8_t StopGameStateEvaluation::metricCheck(const world::World* world, const world::Field* field) const noexcept {
    return GameStateManager::getCurrentGameState().getStrategyName() == "stop" ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation