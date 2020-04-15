//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_BASEINVARIANT_H
#define RTT_BASEINVARIANT_H

#include "world_new/World.hpp"

namespace rtt::ai::stp::invariant {
class BaseInvariant {
   public:
    [[nodiscard]] bool checkInvariant(world_new::view::WorldDataView world, const Field *field, const uint8_t conversionMargin = 127) const noexcept;
    [[nodiscard]] virtual uint8_t metricCheck(world_new::view::WorldDataView world, const Field *field) const noexcept = 0;
    virtual ~BaseInvariant() = default;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BASEINVARIANT_H
