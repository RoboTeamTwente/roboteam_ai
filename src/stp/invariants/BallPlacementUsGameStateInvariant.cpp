//
// Created by timovdk on 4/14/20.
//

#include "stp/invariants/BallPlacementUsGameStateInvariant.h"

#include <utilities/GameStateManager.hpp>

namespace rtt::ai::stp::invariant {
uint8_t BallPlacementUsGameStateInvariant::metricCheck(world_new::view::WorldDataView, const Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getRuleSet().title == "ballplacement_us" ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::invariant