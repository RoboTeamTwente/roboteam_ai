//
// Created by timovdk on 4/20/20.
//

#include "stp/invariants/game_states/BallPlacementThemGameStateInvariant.h"
#include <utilities/GameStateManager.hpp>

namespace rtt::ai::stp::invariant {
uint8_t BallPlacementThemGameStateInvariant::metricCheck(world::view::WorldDataView, const world::Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getStrategyName() == "ball_placement_them" ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::invariant