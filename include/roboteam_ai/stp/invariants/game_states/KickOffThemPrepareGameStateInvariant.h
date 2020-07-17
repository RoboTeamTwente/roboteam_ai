//
// Created by jordi on 28-04-20.
//

#ifndef RTT_KICKOFFTHEMPREPAREGAMESTATEINVARIANT_H
#define RTT_KICKOFFTHEMPREPAREGAMESTATEINVARIANT_H

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {

/**
 * Invariant for the game state kick off them prepare
 */
class KickOffThemPrepareGameStateInvariant : public BaseInvariant {
 public:
  [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept override;

  const char* getName() override { return "gs::KickOffThem"; }
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_KICKOFFUSPREPAREGAMESTATEINVARIANT_H
