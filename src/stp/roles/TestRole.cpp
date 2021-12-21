//
// Created by timovdk on 3/10/20.
//

#include "stp/roles/TestRole.h"

#include "stp/tactics/TestTactic.h"
#include "stp/tactics/passive/BlockRobot.h"

namespace rtt::ai::stp {

TestRole::TestRole(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::BlockRobot()};
}
}  // namespace rtt::ai::stp
