//
// Created by jordi on 08-04-20.
//

#include "stp/roles/Keeper.h"

#include <roboteam_utils/Print.h>

#include "stp/tactics/active/GetBall.h"
#include "stp/tactics/KeeperBlockBall.h"
#include "stp/tactics/active/KickAtPos.h"
#include "stp/tactics/active/ChipAtPos.h"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::role {

Keeper::Keeper(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::KeeperBlockBall(), tactic::GetBall(), tactic::ChipAtPos()};
}

Status Keeper::update(StpInfo const& info) noexcept {
    // Failure if the required data is not present
    if (!info.getBall() || !info.getRobot() || !info.getField()) {
        RTT_WARNING("Required information missing in the tactic info")
        return Status::Failure;
    }

    // Stop blocking when ball is in defense area and still, start getting the ball and pass
    bool stopBlockBall = isBallInOurDefenseAreaAndStill(info.getField().value(), info.getBall().value()->getPos(), info.getBall().value()->getVelocity());
    if (stopBlockBall && robotTactics.current_num() == 0) forceNextTactic();

    if (stopBlockBall && robotTactics.current_num() == 1 && info.getRobot().value().hasBall(0.09,0.2)) forceNextTactic();

    currentRobot = info.getRobot();
    // Update the current tactic with the new tacticInfo
    auto status = robotTactics.update(info);

    // Success if the tactic returned success and if all tactics are done
    if (status == Status::Success && robotTactics.finished()) {
        RTT_INFO("ROLE SUCCESSFUL for ", info.getRobot()->get()->getId())
        return Status::Success;
    }

    // Reset the tactic state machine if a tactic failed and the state machine is not yet finished
    if ((status == Status::Failure && !robotTactics.finished()) || shouldRoleReset(stopBlockBall)) {
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

bool Keeper::isBallInOurDefenseAreaAndStill(const world::Field& field, const Vector2& ballPos, const Vector2& ballVel) noexcept {
    bool pointIsInDefenseArea = FieldComputations::pointIsInDefenseArea(field, ballPos, true);
    bool ballIsLayingStill = ballVel.length() < control_constants::BALL_IS_MOVING_SLOW_LIMIT;

    return pointIsInDefenseArea && ballIsLayingStill;
}

bool Keeper::shouldRoleReset(bool isBallInOurDefenseAreaAndStill) noexcept { return robotTactics.current_num() != 0 && !isBallInOurDefenseAreaAndStill; }

}  // namespace rtt::ai::stp::role
