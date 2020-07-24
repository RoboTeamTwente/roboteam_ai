//
// Created by ratoone on 27-03-20.
//

#include "stp/invariants/game_states/HaltGameStateInvariant.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::invariant {
uint8_t HaltGameStateInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field *field) const noexcept {
    return GameStateManager::getCurrentGameState().getStrategyName() == "halt" ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::invariant
