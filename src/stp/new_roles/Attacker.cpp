//
// Created by jordi on 17-03-20.
//

#include "include/roboteam_ai/stp/new_roles/Attacker.h"
#include "include/roboteam_ai/stp/new_tactics/GetBall.h"
#include "include/roboteam_ai/stp/new_tactics/DriveWithBall.h"
#include "include/roboteam_ai/stp/new_tactics/KickAtPos.h"

namespace rtt::ai::stp::role {

Attacker::Attacker(std::string name) : Role(std::move(name)) {
    // create state machine and initializes the first state
    robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::GetBall()/*, tactic::DriveWithBall()*/, tactic::KickAtPos()};
    robotTactics.initialize();
}

StpInfo Attacker::calculateInfoForTactic(StpInfo const &info) noexcept {
    StpInfo tacticInfo = info;

    // Set info about the shot
    tacticInfo.setPosition({SHOOT_TO_POSITION, calculateKickPosition(tacticInfo)});
    tacticInfo.setKickChipType(MAX_SPEED);

    return tacticInfo;
}

bool Attacker::shouldRoleReset(const StpInfo &info) noexcept {
    // Reset Role when robot lost the ball
    return !info.getRobot().value().hasBall();
}

Vector2 Attacker::calculateKickPosition(StpInfo const info) noexcept {
    // Calculate the position of the goal to kick to
    return info.getField()->getTheirGoalCenter();
}

} // namespace rtt::ai::stp::role