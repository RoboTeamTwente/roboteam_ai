//
// Created by jordi on 11-05-20.
//

#include "stp/new_roles/BallGetter.h"

#include "stp/tactics/GetBall.h"

namespace rtt::ai::stp::role {

BallGetter::BallGetter(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall()};
}

}  // namespace rtt::ai::stp::role
