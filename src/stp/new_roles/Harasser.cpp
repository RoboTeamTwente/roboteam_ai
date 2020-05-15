//
// Created by timovdk on 3/27/20.
//

#include "stp/new_roles/Harasser.h"

#include <utility>

#include "stp/new_tactics/BlockBall.h"
#include "stp/new_tactics/Intercept.h"

namespace rtt::ai::stp::role {

Harasser::Harasser(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::BlockBall(), tactic::Intercept()};
}
}  // namespace rtt::ai::stp::role