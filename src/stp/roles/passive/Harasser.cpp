//
// Created by timovdk on 3/27/20.
//

#include "stp/roles/passive/Harasser.h"

#include "stp/tactics/passive/BlockBall.h"
#include "stp/tactics/Intercept.h"

namespace rtt::ai::stp::role {

Harasser::Harasser(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::BlockBall(), tactic::Intercept()};
}
}  // namespace rtt::ai::stp::role