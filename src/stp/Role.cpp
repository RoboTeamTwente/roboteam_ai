//
// Created by john on 3/9/20.
//

#include "include/roboteam_ai/stp/Role.hpp"

namespace rtt::ai::stp {
Status Role::update(StpInfo const& info) noexcept {
    // Failure if the required data is not present
    if (!info.getBall() || !info.getRobot() || !info.getField()) {
        RTT_WARNING("Required information missing in the tactic info");
        return Status::Failure;
    }

    // Update the current tactic with the new tacticInfo
    auto status = robotTactics.update(info);

    // Success if the tactic returned success and if all tactics are done
    if (status == Status::Success && robotTactics.finished()) {
        RTT_INFO("ROLE SUCCESSFUL for ", info.getRobot()->get()->getId())
        return Status::Success;
    }

    // Reset the tactic state machine if a tactic failed
    if (status == Status::Failure) {
        RTT_INFO("State Machine reset for current role for ID = ", info.getRobot()->get()->getId())
        // Reset all the Skills state machines
        for (auto& tactic : robotTactics) {
            tactic->reset();
        }
        // Reset Tactics state machine
        robotTactics.reset();
    }

    // Success if waiting and tactics are finished
    // Waiting if waiting but not finished
    if (status == Status::Waiting) {
        if (robotTactics.finished()) {
            return Status::Success;
        }
        return Status::Waiting;
    }

    // Return running by default
    return Status::Running;
}

bool Role::finished() const noexcept { return robotTactics.finished(); }

}  // namespace rtt::ai::stp
