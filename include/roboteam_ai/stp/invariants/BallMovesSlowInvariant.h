//
// Created by timovdk on 4/3/20.
//

#ifndef RTT_BALLMOVESSLOWINVARIANT_H
#define RTT_BALLMOVESSLOWINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallMovesSlowInvariant : public BaseInvariant {
    [[nodiscard]] bool checkInvariant(world_new::view::WorldDataView world, const Field *field) const noexcept override;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLMOVESSLOWINVARIANT_H
