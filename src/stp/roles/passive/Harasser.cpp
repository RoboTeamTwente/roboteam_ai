//
// Created by timovdk on 3/27/20.
//

#include "stp/roles/passive/Harasser.h"

#include "stp/tactics/Intercept.h"
#include "stp/tactics/passive/BlockRobot.h"

namespace rtt::ai::stp::role {

Harasser::Harasser(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::BlockRobot()};
}
}  // namespace rtt::ai::stp::role