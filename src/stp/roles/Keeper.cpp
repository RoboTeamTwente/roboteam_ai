//
// Created by jordi on 08-04-20.
//

#include "stp/roles/Keeper.h"

#include <roboteam_utils/Print.h>

#include "stp/tactics/KeeperBlockBall.h"

namespace rtt::ai::stp::role {

Keeper::Keeper(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::KeeperBlockBall()};
}
}  // namespace rtt::ai::stp::role
