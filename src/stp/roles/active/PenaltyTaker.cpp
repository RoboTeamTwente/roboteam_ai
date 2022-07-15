//
// Created by robopc on 15-7-22.
//

#include "stp/roles/active/PenaltyTaker.h"

#include "stp/tactics/active/KickAtPos.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/DriveWithBall.h"
#include "stp/tactics/passive/Formation.h"

namespace rtt::ai::stp::role {

PenaltyTaker::PenaltyTaker(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::KickAtPos(), tactic::Formation()};
}
}  // namespace rtt::ai::stp::role