//
// Created by Jesse on 23-06-20.
/// Rotates (with dribbler) the robot if needed to target angle then calculates the power to SHOOT the BALL with.
// TODO-Max compare to ChipAtPos

/// ACTIVE

#include "stp/tactics/active/ShootAtPos.h"

#include <roboteam_utils/Print.h>

#include "control/ControlUtils.h"
#include "stp/constants/ControlConstants.h"
#include "stp/skills/Rotate.h"
#include "stp/skills/Shoot.h"

namespace rtt::ai::stp::tactic {

ShootAtPos::ShootAtPos() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Rotate(), skill::Shoot()};
}

std::optional<StpInfo> ShootAtPos::calculateInfoForSkill(StpInfo const &info) noexcept {
    if (info.getKickOrChip() == stp::KickOrChip::KICK) {
        return calculateInfoForKick(info);
    } else if (info.getKickOrChip() == stp::KickOrChip::CHIP) {
        return calculateInfoForChip(info);
    } else {
        RTT_ERROR("No ShootType is set, kicking by default")
        return calculateInfoForKick(info);
    }
}

std::optional<StpInfo> ShootAtPos::calculateInfoForKick(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getPositionToShootAt() || !skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    // Calculate the angle the robot needs to aim
    double angleToTarget = (info.getPositionToShootAt().value() - info.getRobot().value()->getPos()).angle();
    skillStpInfo.setAngle(angleToTarget);

    // Calculate the distance and the kick force
    double distanceBallToTarget = (info.getBall()->get()->position - info.getPositionToShootAt().value()).length();
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineKickForce(distanceBallToTarget, skillStpInfo.getShotType()));

    // Set the dribblerSpeed
    skillStpInfo.setDribblerSpeed(100);

    return skillStpInfo;
}

std::optional<StpInfo> ShootAtPos::calculateInfoForChip(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getPositionToShootAt() || !skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    // Calculate the angle the robot needs to aim
    double angleToTarget = (info.getPositionToShootAt().value() - info.getRobot().value()->getPos()).angle();
    skillStpInfo.setAngle(angleToTarget);

    // Calculate the distance and the chip force
    double distanceBallToTarget = (info.getBall()->get()->position - info.getPositionToShootAt().value()).length();
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineChipForce(distanceBallToTarget, skillStpInfo.getShotType()));

    skillStpInfo.setDribblerSpeed(100);

    return skillStpInfo;
}

bool ShootAtPos::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

bool ShootAtPos::isTacticFailing(const StpInfo &info) noexcept {
    // Fail tactic if:
    // robot doesn't have the ball or if there is no shootTarget
    return !info.getRobot().value()->hasBall() || !info.getPositionToShootAt();
}

bool ShootAtPos::shouldTacticReset(const StpInfo &info) noexcept {
    // Reset when angle is wrong outside of the rotate skill, reset to rotate again
    if (skills.current_num() != 0) {
        double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
        return info.getRobot().value()->getAngle().shortestAngleDiff(info.getAngle()) > errorMargin;
    }
    return false;
}

const char *ShootAtPos::getName() { return "Shoot At Pos"; }

}  // namespace rtt::ai::stp::tactic
