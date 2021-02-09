//
// Created by jessevw on 17.03.20.
//

#include "stp/roles/PassReceiver.h"

#include "stp/tactics/Receive.h"

namespace rtt::ai::stp::role {

PassReceiver::PassReceiver(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Receive()};
}
}  // namespace rtt::ai::stp::role