//
// Created by Jaro & Floris on 18-05-21.
//

#include "stp/roles/DemoKeeper.h"

#include <roboteam_utils/Print.h>
#include <stp/tactics/Formation.h>
#include "stp/tactics/KeeperBlockBall.h"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::role {

    DemoKeeper::DemoKeeper(std::string name) : Role(std::move(name)) {
        // create state machine and initializes the first state
        robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Formation(), tactic::KeeperBlockBall()};
    }

    Status DemoKeeper::update(StpInfo const& info) noexcept {
        // Failure if the required data is not present
        if (!info.getBall() || !info.getRobot() || !info.getField()) {
            RTT_WARNING("Required information missing in the tactic info")
            return Status::Failure;
        }

        if (info.getBall().value()->getVelocity().length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT) {
            forceNextTactic();
        }

        if (info.getBall().value()->getVelocity().length() < control_constants::BALL_IS_MOVING_SLOW_LIMIT) {
            robotTactics.reset();
        }

        currentRobot = info.getRobot();
        // Update the current tactic with the new tacticInfo
        auto status = robotTactics.update(info);

        // Success if the tactic returned success and if all tactics are done
        if (status == Status::Success && robotTactics.finished()) {
            RTT_INFO("ROLE SUCCESSFUL for ", info.getRobot()->get()->getId())
            return Status::Success;
        }

        // Reset the tactic state machine if a tactic failed and the state machine is not yet finished
        if ((status == Status::Failure && !robotTactics.finished())) {
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

}  // namespace rtt::ai::stp::role
