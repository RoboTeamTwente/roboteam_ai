//
// Created by jesse on 07-04-20.
//

#include "stp/new_tactics/GetBall.h"
#include "stp/new_roles/BallPlacer.h"
#include "stp/new_tactics/DriveWithBall.h"
#include "stp/new_tactics/AvoidBall.h"

namespace rtt::ai::stp::role {

    BallPlacer::BallPlacer(std::string name) : Role(std::move(name)) {
        // create state machine and initializes the first state
        robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::DriveWithBall(), tactic::AvoidBall()};
}

} // namespace rtt::ai::stp::role
