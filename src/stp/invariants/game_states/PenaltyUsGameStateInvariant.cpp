//
// Created by jordi on 28-04-20.
//

#include "stp/invariants/game_states/PenaltyUsGameStateInvariant.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::invariant {

uint8_t PenaltyUsGameStateInvariant::metricCheck(world::view::WorldDataView, const world::Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getStrategyName() == "penalty_us" ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::invariant