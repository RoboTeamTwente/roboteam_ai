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

    // Check if the skills are all finished
    if (robotTactics.finished()) {
        RTT_INFO("ROLE SUCCESSFUL for ", info.getRobot()->get()->getId())
        return Status::Success;
    }

    // Update tactic info
    auto tacticInfo = calculateInfoForTactic(info);

    // Reset the role
    if (robotTactics.current_num() != 0 && shouldRoleReset(tacticInfo)) {
        RTT_INFO("State Machine reset for current role for ID = ", tacticInfo.getRobot()->get()->getId())
        // Reset all the Tactics state machines
        for (auto& tactic : robotTactics) {
            tactic->reset();
        }
        // Reset Role state machine
        robotTactics.reset();
    }

    // Update the state machine of tactics with the TacticInfo from Play
    currentTacticStatus = robotTactics.update(tacticInfo);
    return currentTacticStatus;
}

bool Role::finished() const noexcept { return robotTactics.finished(); }

}  // namespace rtt::ai::stp
