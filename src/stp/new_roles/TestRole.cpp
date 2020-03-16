//
// Created by timovdk on 3/10/20.
//

#include <roboteam_utils/Print.h>
#include <stp/new_roles/TestRole.h>
#include <stp/new_tactics/TestTactic.h>
#include <stp/new_tactics/KickAtPos.h>

#include <utility>

namespace rtt::ai::stp {

TestRole::TestRole(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::KickAtPos()};
    robotTactics.initialize();
}
}  // namespace rtt::ai::stp