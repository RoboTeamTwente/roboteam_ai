//
// Created by timovdk on 3/13/20.
//

#include "stp/new_tactics/ChipAtPos.h"

#include <roboteam_utils/Print.h>
#include <stp/new_skills/Chip.h>
#include <stp/new_skills/Rotate.h>
#include <include/roboteam_ai/utilities/Constants.h>

namespace rtt::ai::stp::tactic {

ChipAtPos::ChipAtPos() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Rotate(), skill::Chip()};
}

void ChipAtPos::onInitialize() noexcept {}

void ChipAtPos::onUpdate(Status const &status) noexcept {}

void ChipAtPos::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

std::optional<StpInfo> ChipAtPos::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if(!skillStpInfo.getPositionToShootAt() || !skillStpInfo.getRobot() || !skillStpInfo.getBall() || !skillStpInfo.getKickChipType()) return std::nullopt;

    // Calculate the angle the robot needs to aim
    double angleToTarget = (info.getPositionToShootAt().value() - info.getRobot().value()->getPos()).angle();
    skillStpInfo.setAngle(angleToTarget);

    // Calculate the distance and the chip force
    double distanceBallToTarget = (info.getBall()->get()->getPos() - info.getPositionToShootAt().value()).length();
    skillStpInfo.setKickChipVelocity(determineChipForce(distanceBallToTarget, skillStpInfo.getKickChipType()));

    // When rotating, we need to dribble to keep the ball, but when chipping we don't
    if (skills.current_num() == 0) {
        skillStpInfo.setDribblerSpeed(30);
    }

    return skillStpInfo;
}

double ChipAtPos::determineChipForce(double distance, KickChipType desiredBallSpeedType) noexcept {
    // TODO: TUNE these factors need tuning
    // Increase these factors to decrease chip velocity
    // Decrease these factors to increase chip velocity
    const double TARGET_FACTOR{1};
    const double GRSIM_TARGET_FACTOR{1.3};
    const double PASS_FACTOR{0.8};
    const double GRSIM_PASS_FACTOR{1.1};

    if (desiredBallSpeedType == MAX) return stp::control_constants::MAX_CHIP_POWER;

    double limitingFactor{};
    // Pick the right limiting factor based on ballSpeedType and whether we use GRSIM or not
    if (desiredBallSpeedType == PASS) {
        Constants::GRSIM() ? limitingFactor = GRSIM_PASS_FACTOR : limitingFactor = PASS_FACTOR;
    } else if (desiredBallSpeedType == TARGET) {
        Constants::GRSIM() ? limitingFactor = GRSIM_TARGET_FACTOR : limitingFactor = TARGET_FACTOR;
    } else {
        RTT_ERROR("No valid ballSpeedType, chip velocity set to 0")
        return 0;
    }

    // TODO: TUNE this function might need to change
    // TODO: TUNE chip related constants might need tuning
    // Calculate the velocity based on this function with the previously set limitingFactor
    auto velocity = sqrt(distance) * stp::control_constants::MAX_CHIP_POWER / (sqrt(stp::control_constants::MAX_POWER_CHIP_DISTANCE) * limitingFactor);

    // Make sure velocity is always between MIN_CHIP_POWER and MAX_CHIP_POWER
    return std::clamp(velocity, stp::control_constants::MIN_CHIP_POWER, stp::control_constants::MAX_CHIP_POWER);
}

bool ChipAtPos::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

bool ChipAtPos::isTacticFailing(const StpInfo &info) noexcept {
    // Fail tactic if:
    // robot doesn't have the ball or if there is no shootTarget
    // But only check when we are not chipping
    if(skills.current_num() != 1) {
        return !info.getRobot()->hasBall() || !info.getPositionToShootAt();
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

const char *ChipAtPos::getName() {
    return "Chip At Pos";
}

}  // namespace rtt::ai::stp::tactic
