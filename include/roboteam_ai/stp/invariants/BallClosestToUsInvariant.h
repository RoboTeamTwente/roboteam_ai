//
// Created by roboteam on 23/6/20.
//

#ifndef RTT_BALLCLOSESTTOUSINVARIANT_H
#define RTT_BALLCLOSESTTOUSINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallClosestToUsInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept override;

    const char* getName() override { return "BallClosestToUs"; }
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLCLOSESTTOUSINVARIANT_H
