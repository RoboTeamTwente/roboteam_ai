//
// Created by jaro on 30-10-20.
//

#ifndef RTT_BALLONTHEIRSIDEINVARIANT_H
#define RTT_BALLONTHEIRSIDEINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
    class BallOnTheirSideInvariant : public BaseInvariant {
    public:
        [[nodiscard]] uint8_t metricCheck(world::view::WorldDataView world, const world::Field* field) const noexcept override;

        const char* getName() override { return "BallOnTheirSideInvariant"; }
    };
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLONTHEIRSIDEINVARIANT_H
