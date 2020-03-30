//
// Created by ratoone on 27-03-20.
//

#ifndef RTT_BASEINVARIANT_H
#define RTT_BASEINVARIANT_H

#include "world_new/World.hpp"

namespace rtt::ai::stp::invariant {
class BaseInvariant {
public:
    [[nodiscard]] virtual bool checkInvariant(world_new::view::WorldDataView world, const Field *field) const noexcept = 0;

    virtual ~BaseInvariant() = default;
};
}

#endif //RTT_BASEINVARIANT_H
