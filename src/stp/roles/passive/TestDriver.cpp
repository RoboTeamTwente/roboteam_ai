//
// Created by agata on 29/06/2022.
//

#include "stp/roles/passive/TestDriver.h"

#include "stp/tactics/passive/GoLeft.h"
#include "stp/tactics/passive/GoRight.h"

namespace rtt::ai::stp::role {

TestDriver::TestDriver(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GoLeft(), tactic::GoRight()};
}

Status TestDriver::update(StpInfo const& info) noexcept {
    currentRobot = info.getRobot();
    // Update the current tactic with the new tacticInfo
    auto status = robotTactics.update(info);

    if (status == Status::Success && robotTactics.finished()) {
        for (auto& tactic : robotTactics) {
            tactic->reset();
        }
        // Reset Tactics state machine
        robotTactics.reset();
    }

    return Status::Running;
}
}  // namespace rtt::ai::stp::role