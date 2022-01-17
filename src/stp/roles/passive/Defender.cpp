//
// Created by jordi on 26-03-20.
/// TODO-Max Fix Intercept usage in play (mimics "Oppertunic Behavior")
//

#include "stp/roles/passive/Defender.h"

#include "stp/tactics/Intercept.h"
#include "stp/tactics/passive/BlockRobot.h"

namespace rtt::ai::stp::role {

Defender::Defender(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::BlockRobot()};
}
}  // namespace rtt::ai::stp::role
