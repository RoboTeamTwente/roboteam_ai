//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_WEHAVEBALLINVARIANT_H
#define RTT_WEHAVEBALLINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class WeHaveBallInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field *field) const noexcept override;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_WEHAVEBALLINVARIANT_H
