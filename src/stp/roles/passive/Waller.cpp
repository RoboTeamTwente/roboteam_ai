//
// Created by maxl on 15-02-21.
//

#include "stp/tactics/passive/Walling.h"
#include "stp/roles/passive/Waller.h"

namespace rtt::ai::stp::role {

    Waller::Waller(std::string name) : Role(std::move(name)) {
        // create state machine and initializes the first state
        robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Walling()};
    }
}  // namespace rtt::ai::stp::role