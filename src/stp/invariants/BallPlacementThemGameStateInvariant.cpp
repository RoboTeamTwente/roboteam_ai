//
// Created by timovdk on 4/20/20.
//

#include "stp/invariants/BallPlacementThemGameStateInvariant.h"

#include <utilities/GameStateManager.hpp>

namespace rtt::ai::stp::invariant {
uint8_t BallPlacementThemGameStateInvariant::metricCheck(world_new::view::WorldDataView, const Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getRuleSet().title == "ballplacement_them" ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::invariant