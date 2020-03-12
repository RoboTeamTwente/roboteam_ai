//
// Created by timovdk on 3/10/20.
//

#include <roboteam_utils/Print.h>
#include <stp/new_roles/TestRole.h>
#include <stp/new_tactics/TestTactic.h>

#include <utility>
#include <stp/new_tactics/BlockRobot.h>
namespace rtt::ai::stp {

TestRole::TestRole(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{TestTactic()};
    robotTactics.initialize();
}
}  // namespace rtt::ai::stp