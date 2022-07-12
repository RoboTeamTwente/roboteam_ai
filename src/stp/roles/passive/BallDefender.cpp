//
// Created by jordi on 26-03-20.
/// TODO-Max Fix Intercept usage in play (mimics "Oppertunic Behavior")
//

#include "stp/roles/passive/BallDefender.h"

#include "stp/tactics/passive/BlockBall.h"

namespace rtt::ai::stp::role {

BallDefender::BallDefender(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::BlockBall()};
}
}  // namespace rtt::ai::stp::role
