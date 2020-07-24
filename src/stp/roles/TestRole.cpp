//
// Created by timovdk on 3/10/20.
//

#include "stp/roles/TestRole.h"

#include "stp/tactics/TestTactic.h"

namespace rtt::ai::stp {

TestRole::TestRole(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{TestTactic()};
}
}  // namespace rtt::ai::stp
