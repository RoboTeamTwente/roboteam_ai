//
// Created by tijmen on 24-04-22.
//

#include "stp/roles/active/Harasser.h"

#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/passive/Formation.h"

namespace rtt::ai::stp::role {

Harasser::Harasser(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Formation(), tactic::GetBall()};
}
}  // namespace rtt::ai::stp::role