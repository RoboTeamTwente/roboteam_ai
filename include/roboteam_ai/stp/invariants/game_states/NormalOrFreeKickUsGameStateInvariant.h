//
// Created by ratoone on 15-05-20.
//

#ifndef RTT_NORMALORFREEKICKUSGAMESTATEINVARIANT_H
#define RTT_NORMALORFREEKICKUSGAMESTATEINVARIANT_H

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {

class NormalOrFreeKickUsGameStateInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::NormalOrFreeKickUs"; }
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_NORMALORFREEKICKUSGAMESTATEINVARIANT_H
