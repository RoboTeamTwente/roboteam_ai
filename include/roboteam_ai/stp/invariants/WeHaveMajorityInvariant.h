//
// Created by luukkn on 21-04-20.
//

#ifndef RTT_WEHAVEMAJORITYINVARIANT_H
#define RTT_WEHAVEMAJORITYINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
    class WeHaveMajorityInvariant : public BaseInvariant {
            public:
            [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field *field) const noexcept override;

    };
}   // namespace rtt::ai::stp::invariant

#endif //RTT_WEHAVEMAJORITYINVARIANT_H
