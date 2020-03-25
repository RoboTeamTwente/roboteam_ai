//
// Created by jessevw on 24.03.20.
//

#include "include/roboteam_ai/stp/new_roles/Halt.h"
#include <roboteam_utils/Print.h>

#include <utility>
#include <include/roboteam_ai/stp/new_tactics/Halt.h>

namespace rtt::ai::stp::role {

    Halt::Halt(std::string name) : Role(std::move(name)) {
        // create state machine and initializes the first state
        robotTactics = collections::state_machine<Tactic, Status, StpInfo>{rtt::ai::stp::tactic::Halt()};
        robotTactics.initialize();
    }
}  // namespace rtt::ai::stp::role
