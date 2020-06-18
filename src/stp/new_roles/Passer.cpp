//
// Created by jessevw on 17.03.20.
//

#include "stp/new_roles/Passer.h"

#include "stp/new_tactics/GetBallInDirection.h"
#include "stp/new_tactics/KickAtPos.h"

namespace rtt::ai::stp::role {

Passer::Passer(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBallInDirection(), tactic::KickAtPos()};
}
}  // namespace rtt::ai::stp::role
