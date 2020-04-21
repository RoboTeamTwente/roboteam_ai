//
// Created by timovdk on 4/14/20.
//

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {
bool BaseInvariant::checkInvariant(world_new::view::WorldDataView world, const world::Field *field, const uint8_t cutOff) const noexcept {
    return metricCheck(world, field) >= cutOff;
}
}  // namespace rtt::ai::stp::invariant