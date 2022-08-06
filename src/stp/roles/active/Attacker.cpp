//
// Created by jordi on 17-03-20.
/// TODO-Max change to ShooterGoal
//

#include "stp/roles/active/Attacker.h"

#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/active/OrbitKick.h"

namespace rtt::ai::stp::role {

Attacker::Attacker(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::OrbitKick()};
}
}  // namespace rtt::ai::stp::role
