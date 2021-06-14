//
// Created by timovdk on 3/27/20.
//

#include "include/roboteam_ai/stp/roles/passive/Formation.h"

#include "include/roboteam_ai/stp/tactics/passive/Formation.h"

namespace rtt::ai::stp::role {

Formation::Formation(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Formation()};
}
}  // namespace rtt::ai::stp::role
