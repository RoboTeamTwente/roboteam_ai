//
// Created by floris on 08-06-21.
//

#include "include/roboteam_ai/stp/roles/HardwareChallenge1.h"

#include "stp/tactics/GetBehindBallInDirection.h"
#include "stp/tactics/active/DriveWithBall.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/ShootAtPos.h"

namespace rtt::ai::stp::role {

HardwareChallenge1::HardwareChallenge1(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(),tactic::DriveWithBall(), tactic::ShootAtPos()};
}
}  // namespace rtt::ai::stp::role
