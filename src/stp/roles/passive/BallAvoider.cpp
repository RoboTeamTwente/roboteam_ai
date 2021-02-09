//
// Created by jesse on 30-04-20.
//

#include "include/roboteam_ai/stp/roles/passive/BallAvoider.h"

#include "include/roboteam_ai/stp/tactics/passive/AvoidBall.h"

namespace rtt::ai::stp::role {

BallAvoider::BallAvoider(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::AvoidBall()};
}
}  // namespace rtt::ai::stp::role
