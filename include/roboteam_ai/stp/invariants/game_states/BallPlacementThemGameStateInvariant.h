//
// Created by timovdk on 4/20/20.
//

#ifndef RTT_BALLPLACEMENTTHEMGAMESTATEINVARIANT_H
#define RTT_BALLPLACEMENTTHEMGAMESTATEINVARIANT_H

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallPlacementThemGameStateInvariant : public BaseInvariant {
 public:
  [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept override;

  const char* getName() override { return "gs::BallPlacementThem"; }
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLPLACEMENTTHEMGAMESTATEINVARIANT_H
