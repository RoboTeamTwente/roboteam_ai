//
// Created by timovdk on 3/27/20.
//

#include "stp/roles/passive/Formation.h"

#include "stp/tactics/passive/Formation.h"

namespace rtt::ai::stp::role {

Formation::Formation(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Formation()};
}
}  // namespace rtt::ai::stp::role
