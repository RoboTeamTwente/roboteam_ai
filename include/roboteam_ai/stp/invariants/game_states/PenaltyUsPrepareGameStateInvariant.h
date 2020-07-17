//
// Created by jordi on 28-04-20.
//

#ifndef RTT_PENALTYUSPREPAREGAMESTATEINVARIANT_H
#define RTT_PENALTYUSPREPAREGAMESTATEINVARIANT_H

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {

/**
 * Invariant for the game state penalty us prepare
 */
class PenaltyUsPrepareGameStateInvariant : public BaseInvariant {
 public:
  [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept override;

  const char* getName() override { return "gs::PenaltyUsPrepare"; }
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_PENALTYUSPREPAREGAMESTATEINVARIANT_H
