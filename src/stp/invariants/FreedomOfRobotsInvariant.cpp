//
// Created by jordi on 21-04-20.
//

#include "stp/invariants/FreedomOfRobotsInvariant.h"

namespace rtt::ai::stp::invariant {

FreedomOfRobotsInvariant::FreedomOfRobotsInvariant() noexcept {

}

uint8_t FreedomOfRobotsInvariant::metricCheck(world_new::view::WorldDataView world, const Field* field) const noexcept {

}

uint8_t FreedomOfRobotsInvariant::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }

}  // namespace rtt::ai::stp::invariant