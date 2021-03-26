//
// Created by roboteam on 23/6/20.
/// T/F Invariant if FRIENDLY is closest to BALL
//

#include "stp/evaluations/global/BallClosestToUsGlobalEvaluation.h"

namespace rtt::ai::stp::evaluation {

uint8_t BallClosestToUsGlobalEvaluation::metricCheck(world::view::WorldDataView world, const world::Field* field) const noexcept {
    auto closestBall = world.getRobotClosestToBall();
    if (closestBall && closestBall->get()->getTeam() == world::us) {
        return control_constants::FUZZY_TRUE;
    } else
        return control_constants::FUZZY_FALSE;
}
}  // namespace rtt::ai::stp::evaluation
