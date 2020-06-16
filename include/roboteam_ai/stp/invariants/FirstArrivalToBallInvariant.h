//
// Created by jordi on 13-05-20.
//

#ifndef RTT_FIRSTARRIVALTOBALLINVARIANT_H
#define RTT_FIRSTARRIVALTOBALLINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {

class FirstArrivalToBallInvariant : public BaseInvariant {
public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field *field) const noexcept override;

    const char* getName() override
    {
        return "FirstArrivalToBallInvariant";
    }
};

} // namespace rtt::ai::stp::invariant

#endif // RTT_FIRSTARRIVALTOBALLINVARIANT_H
