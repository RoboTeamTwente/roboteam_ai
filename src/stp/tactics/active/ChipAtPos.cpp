//
// Created by timovdk on 3/13/20.
/// Rotates (with dribbler) the robot if needed to target angle then calculates the power to CHIP the BALL with.
//
/// ACTIVE
//

#include "stp/tactics/active/ChipAtPos.h"

#include "control/ControlUtils.h"
#include "stp/constants/ControlConstants.h"
#include "stp/skills/Chip.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

ChipAtPos::ChipAtPos() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Rotate(), skill::Chip()};
}

std::optional<StpInfo> ChipAtPos::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getPositionToShootAt() || !skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    // Calculate the angle the robot needs to aim
    double angleToTarget = (info.getPositionToShootAt().value() - info.getRobot().value()->getPos()).angle();
    skillStpInfo.setAngle(angleToTarget);

    // Calculate the distance and the chip force
    double distanceBallToTarget = (info.getBall()->get()->position - info.getPositionToShootAt().value()).length();
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineChipForce(distanceBallToTarget, skillStpInfo.getShotType()));

    // When rotating, we need to dribble to keep the ball, but when chipping we don't
    if (skills.current_num() == 0) {
        skillStpInfo.setDribblerSpeed(100);
    }

    return skillStpInfo;
}

bool ChipAtPos::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

bool ChipAtPos::isTacticFailing(const StpInfo &info) noexcept {
    // Fail tactic if:
    // robot doesn't have the ball or if there is no shootTarget
    // But only check when we are not chipping
    if (skills.current_num() != 1) {
        return !info.getRobot().value()->hasBall() || !info.getPositionToShootAt();
    }
    return false;
}

bool ChipAtPos::shouldTacticReset(const StpInfo &info) noexcept {
    // Reset when angle is wrong outside of the rotate skill, reset to rotate again
    if (skills.current_num() != 0) {
        double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
        return info.getRobot().value()->getAngle().shortestAngleDiff(info.getAngle()) > errorMargin;
    }
    return false;
}

const char *ChipAtPos::getName() { return "Chip At Pos"; }

}  // namespace rtt::ai::stp::tactic
