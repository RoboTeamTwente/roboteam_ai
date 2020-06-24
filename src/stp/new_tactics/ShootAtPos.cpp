//
// Created by Jesse on 23-06-20.
//

#include "stp/new_tactics/ShootAtPos.h"

#include <control/ControlUtils.h>
#include <stp/new_skills/Rotate.h>
#include <stp/new_skills/Shoot.h>

#include "stp/new_tactics/KickAtPos.h"

namespace rtt::ai::stp::tactic {

ShootAtPos::ShootAtPos() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Rotate(), skill::Shoot()};
}

void ShootAtPos::onInitialize() noexcept {}

void ShootAtPos::onUpdate(Status const &status) noexcept {}

void ShootAtPos::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}
std::optional<StpInfo> ShootAtPos::calculateInfoForSkill(StpInfo const &info) noexcept {
    if (info.getShootType() == stp::KickChip::KICK) {
        return calculateInfoForKick(info);
    }
    else if (info.getShootType() == stp::KickChip::CHIP) {
        return calculateInfoForChip(info);
    }
    else {
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
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineKickForce(distanceBallToTarget, skillStpInfo.getKickChipType()));

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
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineChipForce(distanceBallToTarget, skillStpInfo.getKickChipType()));

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
    return !info.getRobot()->hasBall() || !info.getPositionToShootAt();
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
