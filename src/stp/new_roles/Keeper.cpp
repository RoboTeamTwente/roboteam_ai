//
// Created by jordi on 08-04-20.
//

#include "stp/new_roles/Keeper.h"
#include "stp/new_tactics/KickAtPos.h"
#include "stp/new_tactics/GetBall.h"

namespace rtt::ai::stp::role {

Keeper::Keeper(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::BlockBall(), tactic::GetBall(), tactic::KickAtPos()};
}

}  // namespace rtt::ai::stp::role
