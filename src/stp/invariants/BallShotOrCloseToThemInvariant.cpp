//
// Created by jordi on 19-05-20.
//

#include "stp/invariants/BallShotOrCloseToThemInvariant.h"
#include "stp/invariants/BallGotShotInvariant.h"
#include "stp/invariants/BallCloseToThemInvariant.h"

namespace rtt::ai::stp::invariant{

uint8_t BallShotOrCloseToThemInvariant::metricCheck(world::view::WorldDataView world, const world::Field *field) const noexcept {
    auto ballShotOrCloseToThem = BallGotShotInvariant().checkInvariant(world, field) ||
            BallCloseToThemInvariant().checkInvariant(world, field);
    return ballShotOrCloseToThem ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
}

}  // namespace rtt::ai::stp::invariant