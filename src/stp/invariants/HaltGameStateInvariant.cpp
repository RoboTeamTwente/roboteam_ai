//
// Created by ratoone on 27-03-20.
//

#include "stp/invariants/HaltGameStateInvariant.h"
#include <utilities/GameStateManager.hpp>

namespace rtt::ai::stp::invariant {
uint8_t HaltGameStateInvariant::metricCheck(world_new::view::WorldDataView world, const Field *field) const noexcept {
    return GameStateManager::getCurrentGameState().getRuleSet().title == "halt" ? 255 : 0;
}
}  // namespace rtt::ai::stp::invariant
