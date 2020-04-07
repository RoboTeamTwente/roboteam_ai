//
// Created by jesse on 07-04-20.
//

#include <include/roboteam_ai/stp/new_tactics/GetBall.h>
#include "include/roboteam_ai/stp/new_roles/BallPlacer.h"
#include "stp/new_tactics/DriveWithBall.h"

namespace rtt::ai::stp::role {

    BallPlacer::BallPlacer(std::string name) : Role(std::move(name)) {
        // create state machine and initializes the first state
        robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::DriveWithBall()};
    }

} // namespace rtt::ai::stp::role
