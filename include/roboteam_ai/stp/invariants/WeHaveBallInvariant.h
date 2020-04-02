//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_WEHAVEBALLINVARIANT_H
#define RTT_WEHAVEBALLINVARIANT_H

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class WeHaveBallInvariant : public BaseInvariant{
public:
    [[nodiscard]] bool checkInvariant(world_new::view::WorldDataView world, const Field *field) const noexcept override;
};
}

#endif //RTT_WEHAVEBALLINVARIANT_H