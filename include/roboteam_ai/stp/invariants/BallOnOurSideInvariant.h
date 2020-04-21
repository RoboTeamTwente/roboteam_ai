//
// Created by timovdk on 4/14/20.
//

#ifndef RTT_BALLONOURSIDEINVARIANT_H
#define RTT_BALLONOURSIDEINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallOnOurSideInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field *field) const noexcept override;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLONOURSIDEINVARIANT_H