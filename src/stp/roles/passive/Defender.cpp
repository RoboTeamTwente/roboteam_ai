//
// Created by jordi on 26-03-20.
/// TODO-Max Fix Intercept usage in play (mimics "Oppertunic Behavior")
//

#include "stp/roles/passive/Defender.h"

#include "stp/tactics/passive/BlockRobot.h"
#include "stp/tactics/Intercept.h"

namespace rtt::ai::stp::role {

Defender::Defender(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::BlockRobot(), tactic::Intercept()};
}
}  // namespace rtt::ai::stp::role
