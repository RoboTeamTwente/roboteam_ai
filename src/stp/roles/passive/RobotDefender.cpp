//
// Created by timovdk on 3/27/20.
//

#include "stp/roles/passive/RobotDefender.h"

#include "stp/tactics/passive/BlockRobot.h"

namespace rtt::ai::stp::role {

RobotDefender::RobotDefender(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::BlockRobot()};
}
}  // namespace rtt::ai::stp::role