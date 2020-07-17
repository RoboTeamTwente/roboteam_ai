//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_HALTGAMESTATEINVARIANT_H
#define RTT_HALTGAMESTATEINVARIANT_H

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class HaltGameStateInvariant : public BaseInvariant {
 public:
  [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept override;

  const char* getName() override { return "gs::Halt"; }
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_HALTGAMESTATEINVARIANT_H
