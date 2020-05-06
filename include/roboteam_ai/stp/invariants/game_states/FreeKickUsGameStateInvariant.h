//
// Created by jordi on 28-04-20.
//

#ifndef RTT_FREEKICKUSGAMESTATEINVARIANT_H
#define RTT_FREEKICKUSGAMESTATEINVARIANT_H

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {

/**
 * Invariant for the game state free kick us prepare
 */
class FreeKickUsGameStateInvariant : public BaseInvariant {
public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept override;
};

}  // namespace rtt::ai::stp::invariant

#endif //RTT_FREEKICKUSGAMESTATEINVARIANT_H
