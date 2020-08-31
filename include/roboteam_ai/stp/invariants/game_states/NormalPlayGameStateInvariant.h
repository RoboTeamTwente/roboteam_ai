//
// Created by jordi on 28-04-20.
//

#ifndef RTT_NORMALPLAYGAMESTATEINVARIANT_H
#define RTT_NORMALPLAYGAMESTATEINVARIANT_H

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {

/**
 * Invariant for the game state normal play
 */
class NormalPlayGameStateInvariant : public BaseInvariant {
   public:
    [[nodiscard]] uint8_t metricCheck(world::view::WorldDataView world, const world::Field* field) const noexcept override;

    const char* getName() override { return "gs::NormalPlay"; }
};
}  // namespace rtt::ai::stp::invariant

#endif  // RTT_NORMALPLAYGAMESTATEINVARIANT_H
