//
// Created Alexander on 22-04-2022
//

#include "stp/roles/active/FreeKickTaker.h"

#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/GetBehindBallInDirection.h"
#include "stp/tactics/active/KickAtPos.h"
#include "stp/tactics/passive/Formation.h"

namespace rtt::ai::stp::role {

FreeKickTaker::FreeKickTaker(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBehindBallInDirection(), tactic::GetBall(), tactic::KickAtPos(), tactic::Formation()};
}
}  // namespace rtt::ai::stp::role
