//
// Created by jordi on 19-05-20.
//

#include "stp/new_roles/BallReflecter.h"

#include "stp/new_tactics/Receive.h"
#include "stp/new_tactics/KickAtPos.h"

namespace rtt::ai::stp::role {

BallReflecter::BallReflecter(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Receive(), tactic::KickAtPos()};
}

} // namespace rtt::ai::stp::role