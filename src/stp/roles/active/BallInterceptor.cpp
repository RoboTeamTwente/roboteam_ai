//
// Created by alexander on 12-05-22.
//

#include "stp/roles/active/BallInterceptor.h"

#include "stp/tactics/active/Intercept.h"
#include "stp/tactics/active/GetBall.h"

namespace rtt::ai::stp::role {

BallInterceptor::BallInterceptor(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Intercept(), tactic::GetBall()};
}
}  // namespace rtt::ai::stp::role
