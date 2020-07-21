//
// Created by Jesse on 23-06-20.
//

#include "stp/new_tactics/ShootAtPos.h"

#include <control/ControlUtils.h>
#include <stp/new_skills/Rotate.h>
#include <stp/new_skills/Shoot.h>

#include "stp/new_tactics/KickAtPos.h"

#include "roboteam_utils/Print.h"

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
    double distanceBallToTarget = (info.getBall()->get()->getPos() - info.getPositionToShootAt().value()).length();
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineKickForce(distanceBallToTarget, skillStpInfo.getShotType()));

    // When the angle is not within the margin, dribble so we don't lose the ball while rotating
    double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    skillStpInfo.setDribblerSpeed(50);

    return skillStpInfo;
}

std::optional<StpInfo> ShootAtPos::calculateInfoForChip(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getPositionToShootAt() || !skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    // Calculate the angle the robot needs to aim
    double angleToTarget = (info.getPositionToShootAt().value() - info.getRobot().value()->getPos()).angle();
    skillStpInfo.setAngle(angleToTarget);

    // Calculate the distance and the chip force
    double distanceBallToTarget = (info.getBall()->get()->getPos() - info.getPositionToShootAt().value()).length();
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineChipForce(distanceBallToTarget, skillStpInfo.getShotType()));

    // When rotating, we need to dribble to keep the ball, but when chipping we don't
    if (skills.current_num() == 0) {
        skillStpInfo.setDribblerSpeed(30);
    }

    return skillStpInfo;
}

bool ShootAtPos::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

bool ShootAtPos::isTacticFailing(const StpInfo &info) noexcept {
    // Fail tactic if:
    // robot doesn't have the ball or if there is no shootTarget
    auto ballDistance = info.getRobot()->get()->getDistanceToBall() < control_constants::HAS_BALL_DISTANCE_ERROR_MARGIN * 3;

    return (!info.getRobot()->hasBall() && info.getBall()->get()->getFilteredVelocity().length() < control_constants::BALL_STILL_VEL) || !info.getPositionToShootAt();
}

bool ShootAtPos::shouldTacticReset(const StpInfo &info) noexcept {
    // Reset when angle is wrong outside of the rotate skill, reset to rotate again
    if (info.getBall().value()->getVelocity().length() > control_constants::BALL_STILL_VEL) {
        return false;
    }
    if (skills.current_num() != 0) {
        double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
        return info.getRobot().value()->getAngle().shortestAngleDiff(info.getAngle()) > errorMargin;
    }
    return false;
}

const char *ShootAtPos::getName() { return "Shoot At Pos"; }

}  // namespace rtt::ai::stp::tactic