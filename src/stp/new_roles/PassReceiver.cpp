//
// Created by jessevw on 17.03.20.
//

#include "include/roboteam_ai/stp/new_roles/PassReceiver.h"
#include <roboteam_utils/Print.h>
#include <stp/new_roles/TestRole.h>
#include <stp/new_tactics/TestTactic.h>

#include <utility>
#include <stp/new_tactics/BlockRobot.h>
#include <include/roboteam_ai/stp/new_tactics/Receive.h>

namespace rtt::ai::stp {

    PassReceiver::PassReceiver(std::string name) : Role(std::move(name)) {
        // create state machine and initializes the first state
        robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Receive()};
        robotTactics.initialize();
    }

    StpInfo PassReceiver::calculateInfoForTactic(StpInfo const &info) noexcept {
        return info;
    }

    bool PassReceiver::shouldRoleReset(const StpInfo &info) noexcept {
        return currentTacticStatus == Status::Failure;
    }

}  // namespace rtt::ai::stp