//
// Created by timovdk on 4/14/20.
//

#include "stp/invariants/BallCloseToUsInvariant.h"

namespace rtt::ai::stp::invariant {
bool BallCloseToUsInvariant::checkInvariant(world_new::view::WorldDataView world, const Field* field) const noexcept {
    auto& us = world.getUs();
    auto ballPos = world.getBall()->get()->getPos();
    // If the distance between ballpos and robotpos is smaller than BALL_IS_CLOSE constant return true
    return std::any_of(us.begin(), us.end(), [&ballPos](world_new::view::RobotView& robot) { return robot.get()->getPos().dist(ballPos) < control_constants::BALL_IS_CLOSE; });
}
}  // namespace rtt::ai::stp::invariant