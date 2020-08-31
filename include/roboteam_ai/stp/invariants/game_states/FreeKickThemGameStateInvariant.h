//
// Created by jordi on 28-04-20.
//

#ifndef RTT_FREEKICKTHEMGAMESTATEINVARIANT_H
#define RTT_FREEKICKTHEMGAMESTATEINVARIANT_H

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {

/**
 * Invariant for the game state free kick them prepare
 */
class FreeKickThemGameStateInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world::view::WorldDataView world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::FreeKickThem"; }
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_FREEKICKTHEMGAMESTATEINVARIANT_H
