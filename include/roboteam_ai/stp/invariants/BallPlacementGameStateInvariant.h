//
// Created by timovdk on 4/14/20.
//

#ifndef RTT_BALLPLACEMENTGAMESTATEINVARIANT_H
#define RTT_BALLPLACEMENTGAMESTATEINVARIANT_H

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallPlacementGameStateInvariant : public BaseInvariant {
    [[nodiscard]] bool checkInvariant(world_new::view::WorldDataView world, const Field *field) const noexcept override;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLPLACEMENTGAMESTATEINVARIANT_H
