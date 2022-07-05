//
// Created by jesse on 07-04-20.
/// TODO-Max check if T:AvoidBall() can be done otherwise (end of play?)
//

#include "stp/roles/active/BallPlacer.h"

#include <roboteam_utils/Print.h>

#include "stp/tactics/active/DriveWithBall.h"
#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/GetBehindBallInDirection.h"
#include "stp/tactics/passive/AvoidBall.h"
#include "stp/tactics/passive/BallStandBack.h"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::role {

BallPlacer::BallPlacer(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::DriveWithBall(), tactic::BallStandBack(),
                                                                       tactic::AvoidBall()};
}
}  // namespace rtt::ai::stp::role
