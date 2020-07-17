//
// Created by jordi on 28-04-20.
//

#ifndef RTT_KICKOFFTHEMGAMESTATEINVARIANT_H
#define RTT_KICKOFFTHEMGAMESTATEINVARIANT_H

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {

/**
 * Invariant for the game state kick off them
 */
class KickOffThemGameStateInvariant : public BaseInvariant {
 public:
  [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept override;

  const char* getName() override { return "gs::KickOffThem"; }
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_KICKOFFTHEMGAMESTATEINVARIANT_H
