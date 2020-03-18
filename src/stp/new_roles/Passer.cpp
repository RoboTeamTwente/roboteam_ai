//
// Created by jessevw on 17.03.20.
//

#include "include/roboteam_ai/stp/new_roles/Passer.h"
#include <roboteam_utils/Print.h>
#include <stp/new_tactics/TestTactic.h>
#include <utility>
#include <include/roboteam_ai/stp/new_tactics/GetBall.h>
#include <include/roboteam_ai/stp/new_tactics/KickAtPos.h>

namespace rtt::ai::stp {

    Passer::Passer(std::string name) : Role(std::move(name)) {
        // create state machine and initializes the first state
        robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall(), tactic::KickAtPos()};
        robotTactics.initialize();
    }

    StpInfo Passer::calculateInfoForTactic(const StpInfo &info) noexcept {
        return info;
    }

    bool Passer::shouldRoleReset(const StpInfo &info) noexcept {
        return currentTacticStatus == Status::Failure;
    }

}  // namespace rtt::ai::stp