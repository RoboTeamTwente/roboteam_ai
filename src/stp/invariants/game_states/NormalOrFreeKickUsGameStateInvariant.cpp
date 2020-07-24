//
// Created by ratoone on 15-05-20.
//

#include "stp/invariants/game_states/NormalOrFreeKickUsGameStateInvariant.h"

#include "utilities/GameStateManager.hpp"

namespace rtt::ai::stp::invariant {

uint8_t NormalOrFreeKickUsGameStateInvariant::metricCheck(world_new::view::WorldDataView, const world::Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getStrategyName() == "free_kick_us" || GameStateManager::getCurrentGameState().getStrategyName() == "normal_play"
               ? stp::control_constants::FUZZY_TRUE
               : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::invariant