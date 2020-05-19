//
// Created by jesse on 30-04-20.
//

#include "stp/new_roles/BallAvoider.h"
#include "stp/new_tactics/AvoidBall.h"
namespace rtt::ai::stp::role {

    BallAvoider::BallAvoider(std::string name) : Role(std::move(name)) {
        // create state machine and initializes the first state
        robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::AvoidBall()};
    }

} // namespace rtt::ai::stp::role