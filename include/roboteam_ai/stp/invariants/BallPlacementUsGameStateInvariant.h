//
// Created by timovdk on 4/14/20.
//

#ifndef RTT_BALLPLACEMENTUSGAMESTATEINVARIANT_H
#define RTT_BALLPLACEMENTUSGAMESTATEINVARIANT_H

#include <NFParam/Param.h>

#include "BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallPlacementUsGameStateInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const Field* field) const noexcept override;
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLPLACEMENTUSGAMESTATEINVARIANT_H
