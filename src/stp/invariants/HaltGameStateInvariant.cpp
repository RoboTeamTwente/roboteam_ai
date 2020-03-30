//
// Created by ratoone on 27-03-20.
//

#include "stp/invariants/HaltGameStateInvariant.h"

namespace rtt::ai::stp::invariant {
    bool HaltGameStateInvariant::checkInvariant(world_new::view::WorldDataView world) const noexcept {
        GameStateManager::getCurrentGameState().getRuleSet()
    }
}