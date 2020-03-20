//
// Created by john on 3/9/20.
//

#include "include/roboteam_ai/stp/Role.hpp"

namespace rtt::ai::stp {
Status Role::update(StpInfo const& info) noexcept {
    if (!info.getBall() || !info.getRobot() || !info.getField()) {
        RTT_WARNING("Required information missing in the tactic info");
        return Status::Failure;
    }

    // Update the current tactic with the new tacticInfo
    auto status = robotTactics.update(info);

    // Check if the skills are all finished
    if (robotTactics.finished()) {
        RTT_INFO("ROLE SUCCESSFUL for ", info.getRobot()->get()->getId())
        return Status::Success;
    }

    // Reset the role
    if (robotTactics.current_num() != 0 && status == Status::Failure) {
        RTT_INFO("State Machine reset for current role for ID = ", info.getRobot()->get()->getId())
        // Reset all the Tactics state machines
        for (auto& tactic : robotTactics) {
            tactic->reset();
        }
        // Reset Role state machine
        robotTactics.reset();
    }

    return Status::Running;
}

bool Role::finished() const noexcept { return robotTactics.finished(); }

}  // namespace rtt::ai::stp
