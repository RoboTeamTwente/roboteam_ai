//
// Created by jordi on 28-04-20.
//

#ifndef RTT_BALLISFREEINVARIANT_H
#define RTT_BALLISFREEINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {

class BallIsFreeInvariant : public BaseInvariant {
public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field *field) const noexcept override;
};

}  // namespace rtt::ai::stp::invariant

#endif //RTT_BALLISFREEINVARIANT_H
