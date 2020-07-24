//
// Created by jordi on 28-04-20.
//

#ifndef RTT_PENALTYTHEMPREPAREGAMESTATEINVARIANT_H
#define RTT_PENALTYTHEMPREPAREGAMESTATEINVARIANT_H

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {

/**
 * Invariant for the game state penalty them prepare
 */
class PenaltyThemPrepareGameStateInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::PenaltyThemPrepare"; }
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_PENALTYTHEMPREPAREGAMESTATEINVARIANT_H
