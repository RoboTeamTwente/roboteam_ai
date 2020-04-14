//
// Created by timo on 4/2/20.
//

#ifndef RTT_BALLMOVESFASTINVARIANT_H
#define RTT_BALLMOVESFASTINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallMovesFastInvariant : public BaseInvariant {
    [[nodiscard]] bool checkInvariant(world_new::view::WorldDataView world, const Field *field) const noexcept override;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLMOVESFASTINVARIANT_H
