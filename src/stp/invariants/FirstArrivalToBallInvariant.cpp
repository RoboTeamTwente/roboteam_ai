//
// Created by jordi on 13-05-20.
//

#include "stp/invariants/FirstArrivalToBallInvariant.h"

namespace rtt::ai::stp::invariant {

uint8_t FirstArrivalToBallInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field *field) const noexcept {
    // TODO: Improve by taking into account velocity and direction of the robots to calculate ETA
    world_new::Team closestTeam{world_new::Team::both};
        if(world.getRobotClosestToBall()) closestTeam = world.getRobotClosestToBall().value()->getTeam();
    return closestTeam == world_new::us ? control_constants::FUZZY_TRUE : control_constants::FUZZY_FALSE;
}

}  // namespace rtt::ai::stp::invariant