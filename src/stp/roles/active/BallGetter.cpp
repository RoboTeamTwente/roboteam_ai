//
// Created by jordi on 11-05-20.
//

#include <stp/tactics/GetBehindBallInDirection.h>
#include "include/roboteam_ai/stp/roles/active/BallGetter.h"

#include "include/roboteam_ai/stp/tactics/active/GetBall.h"

namespace rtt::ai::stp::role {

BallGetter::BallGetter(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBehindBallInDirection()};
}
}  // namespace rtt::ai::stp::role
