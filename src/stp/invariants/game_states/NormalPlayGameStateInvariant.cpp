//
// Created by jordi on 28-04-20.
//

#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"
#include <utilities/GameStateManager.hpp>

namespace rtt::ai::stp::invariant {

uint8_t NormalPlayGameStateInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field *field) const noexcept {
    return GameStateManager::getCurrentGameState().getStrategyName() == "normal_play" ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}

}  // namespace rtt::ai::stp::invariant