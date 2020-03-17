//
// Created by jordi on 17-03-20.
//

#include "include/roboteam_ai/stp/new_roles/Attacker.h"
#include "include/roboteam_ai/stp/new_tactics/GetBall.h"
#include "include/roboteam_ai/stp/new_tactics/DriveWithBall.h"
#include "include/roboteam_ai/stp/new_tactics/KickAtPos.h"

namespace rtt::ai::stp::role {

Attacker::Attacker(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::DriveWithBall(), tactic::KickAtPos()};
    robotTactics.initialize();
}

} // namespace rtt::ai::stp::role