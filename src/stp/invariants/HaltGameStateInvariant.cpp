//
// Created by ratoone on 27-03-20.
//

#include "stp/invariants/HaltGameStateInvariant.h"
#include <utilities/GameStateManager.hpp>

namespace rtt::ai::stp::invariant {
bool HaltGameStateInvariant::checkInvariant(world_new::view::WorldDataView, const Field *) const noexcept {
    return GameStateManager::getCurrentGameState().getRuleSet().maxRobotVel == 0;
}
}  // namespace rtt::ai::stp::invariant
