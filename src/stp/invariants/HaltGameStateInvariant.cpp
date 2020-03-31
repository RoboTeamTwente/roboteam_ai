//
// Created by ratoone on 27-03-20.
//

#include "stp/invariants/HaltGameStateInvariant.h"

namespace rtt::ai::stp::invariant {
    bool HaltGameStateInvariant::checkInvariant(world_new::view::WorldDataView world, const Field *field) const noexcept {
        return GameStateManager::getCurrentGameState().getRuleSet().maxRobotVel == 0;
    }
}