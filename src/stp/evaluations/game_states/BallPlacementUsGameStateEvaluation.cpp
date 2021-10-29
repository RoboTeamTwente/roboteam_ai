//
// Created by timovdk on 4/14/20.
//

#include "stp/evaluations/game_states/BallPlacementUsGameStateEvaluation.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::evaluation {
uint8_t BallPlacementUsGameStateEvaluation::metricCheck(const world::World *, const world::Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getStrategyName() == "ball_placement_us" ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation