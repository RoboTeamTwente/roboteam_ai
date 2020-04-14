//
// Created by timovdk on 4/14/20.
//

#include "stp/invariants/BallPlacementGameStateInvariant.h"

#include <utilities/GameStateManager.hpp>

namespace rtt::ai::stp::invariant {
bool BallPlacementGameStateInvariant::checkInvariant(world_new::view::WorldDataView, const Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getRuleSet().title == "ballplacement_us" || GameStateManager::getCurrentGameState().getRuleSet().title == "ballplacement_them";
}
}  // namespace rtt::ai::stp::invariant