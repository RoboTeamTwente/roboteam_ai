//
// Created by timovdk on 4/14/20.
//

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {
bool BaseInvariant::checkInvariant(world_new::view::WorldDataView world, const Field *field, const double conversionMargin) const noexcept {
    return metricCheck(world, field) >= conversionMargin;
}
}  // namespace rtt::ai::stp::invariant