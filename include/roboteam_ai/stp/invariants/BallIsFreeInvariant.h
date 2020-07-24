//
// Created by jordi on 28-04-20.
//

#ifndef RTT_BALLISFREEINVARIANT_H
#define RTT_BALLISFREEINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
/**
 * Check if the ball is currently not owned by any team
 */
class BallIsFreeInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept override;

    const char* getName() override { return "BallIsFree"; }
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLISFREEINVARIANT_H
