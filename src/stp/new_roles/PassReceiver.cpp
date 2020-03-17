//
// Created by jessevw on 16.03.20.
//

#include "include/roboteam_ai/stp/new_roles/PassReceiver.h"

#include <roboteam_utils/Print.h>
#include <stp/new_tactics/TestTactic.h>

#include <utility>
#include <include/roboteam_ai/stp/new_tactics/Receive.h>

namespace rtt::ai::stp {

    PassReceiver::PassReceiver(std::string name) : Role(std::move(name)) {
        // create state machine and initializes the first state
        robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Receive()};
        robotTactics.initialize();
    }
}  // namespace rtt::ai::stp