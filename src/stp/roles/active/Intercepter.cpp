
//
// Created by sarah on 02-03-22.
//

#include "stp/roles/active/Intercepter.h"

#include "stp/tactics/Intercept.h"
#include "stp/tactics/passive/Halt.h"

namespace rtt::ai::stp::role {

Intercepter::Intercepter(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Intercept()};
}
}  // namespace rtt::ai::stp