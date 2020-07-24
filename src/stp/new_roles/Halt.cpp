//
// Created by jessevw on 24.03.20.
//

#include "stp/new_roles/Halt.h"

#include <stp/tactics/Halt.h>

namespace rtt::ai::stp::role {

    Halt::Halt(std::string name) : Role(std::move(name)) {
        // create state machine and initializes the first state
        robotTactics = collections::state_machine<Tactic, Status, StpInfo>{rtt::ai::stp::tactic::Halt()};
    }
}  // namespace rtt::ai::stp::role
