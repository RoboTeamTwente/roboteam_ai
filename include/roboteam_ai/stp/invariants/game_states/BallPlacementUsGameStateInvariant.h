//
// Created by timovdk on 4/14/20.
//

#ifndef RTT_BALLPLACEMENTUSGAMESTATEINVARIANT_H
#define RTT_BALLPLACEMENTUSGAMESTATEINVARIANT_H

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {
class BallPlacementUsGameStateInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world::view::WorldDataView world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::BallPlacementUs"; }
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_BALLPLACEMENTUSGAMESTATEINVARIANT_H
