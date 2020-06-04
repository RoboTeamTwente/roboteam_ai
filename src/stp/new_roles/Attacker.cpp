//
// Created by jordi on 17-03-20.
//

#include "stp/new_roles/Attacker.h"
#include "stp/new_tactics/GetBallInDirection.h"
#include "stp/new_tactics/DriveWithBall.h"
#include "stp/new_tactics/KickAtPos.h"
#include "stp/new_tactics/GetBall.h"

namespace rtt::ai::stp::role {

Attacker::Attacker(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBallInDirection()/*, tactic::DriveWithBall()*/, tactic::KickAtPos()};
}

} // namespace rtt::ai::stp::role
