//
// Created by jessevw on 17.03.20.
//

#include "include/roboteam_ai/stp/new_roles/Passer.h"

#include <include/roboteam_ai/stp/new_tactics/GetBall.h>
#include <include/roboteam_ai/stp/new_tactics/KickAtPos.h>
#include <roboteam_utils/Print.h>

#include <utility>

namespace rtt::ai::stp::role {

Passer::Passer(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::KickAtPos()};
}
}  // namespace rtt::ai::stp::role