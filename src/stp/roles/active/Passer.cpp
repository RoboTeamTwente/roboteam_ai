//
// Created by jessevw on 17.03.20.
//

#include "stp/roles/active/Passer.h"

#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/KickAtPos.h"
#include "stp/tactics/passive/Formation.h"

namespace rtt::ai::stp::role {

Passer::Passer(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::KickAtPos(), tactic::Formation()};
}
}  // namespace rtt::ai::stp::role
