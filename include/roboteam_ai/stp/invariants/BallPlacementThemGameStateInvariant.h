//
// Created by timovdk on 4/20/20.
//

#ifndef RTT_BALLPLACEMENTTHEMGAMESTATEINVARIANT_H
#define RTT_BALLPLACEMENTTHEMGAMESTATEINVARIANT_H

#include <NFParam/Param.h>

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallPlacementThemGameStateInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const Field* field) const noexcept override;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLPLACEMENTTHEMGAMESTATEINVARIANT_H
