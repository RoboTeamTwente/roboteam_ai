//
// Created by jordi on 26-03-20.
//

#include "stp/tactics/Intercept.h"
#include "stp/tactics/BlockRobot.h"
#include "stp/new_roles/Defender.h"

namespace rtt::ai::stp::role {

Defender::Defender(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::BlockRobot(), tactic::Intercept()};
    robotTactics.initialize();
}

}
