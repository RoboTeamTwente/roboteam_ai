//
// Created by jesse on 30-04-20.
//

#include "stp/roles/passive/BallAvoider.h"

#include "stp/tactics/passive/AvoidBall.h"
#include "stp/tactics/passive/Formation.h"



namespace rtt::ai::stp::role {

BallAvoider::BallAvoider(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::AvoidBall(), tactic::Formation()};
}
}  // namespace rtt::ai::stp::role
