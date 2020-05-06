//
// Created by jordi on 28-04-20.
//

#ifndef RTT_TIMEOUTGAMESTATEINVARIANT_H
#define RTT_TIMEOUTGAMESTATEINVARIANT_H

#include "stp/invariants/BaseInvariant.h"

namespace rtt::ai::stp::invariant {

/**
 * Invariant for the game state timeout
 */
class TimeOutGameStateInvariant : public BaseInvariant {
public:
    [[nodiscard]] uint8_t metricCheck(world_new::view::WorldDataView world, const world::Field *field) const noexcept override;
};

}  // namespace rtt::ai::stp::invariant

#endif //RTT_TIMEOUTGAMESTATEINVARIANT_H
