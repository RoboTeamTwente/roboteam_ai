//
// Created by Jesse on 23-06-20.
//

#include "stp/skills/Shoot.h"

#include "roboteam_utils/Print.h"
#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::skill {

Status Shoot::onUpdate(const StpInfo &info) noexcept {
    if (info.getKickOrChip() == KickOrChip::CHIP) {
        return onUpdateChip(info);
    } else if (info.getKickOrChip() == KickOrChip::KICK) {
        return onUpdateKick(info);
    } else {
        RTT_ERROR("No ShootType set, kicking by default!")
        return onUpdateKick(info);
    }
}
/// If we kick, this function is called
Status Shoot::onUpdateKick(const StpInfo &info) noexcept {
    // Clamp and set kick velocity
    float kickVelocity = std::clamp(info.getKickChipVelocity(), 0.0, stp::control_constants::MAX_KICK_POWER);

    // Set kick command
    command.kickType = KickType::KICK;
    command.kickSpeed = kickVelocity;

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    double targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

    // Set dribbler speed command
    command.dribblerSpeed = targetDribblerSpeed;

    // Set angle command
    command.targetAngle = info.getRobot().value()->getAngle();

    // Set chip_kick_forced if we can chip but did not chip for MAX_CHIP_ATTEMPTS amount of ticks
    if (shootAttempts > control_constants::MAX_KICK_ATTEMPTS) {
        command.waitForBall = false;
        shootAttempts = 0;
    } else {
        command.waitForBall = true;
    }

    // set command ID
    command.id = info.getRobot().value()->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    if (!info.getRobot().value()->hasBall()) {
        shootAttempts = 0;
        return Status::Success;
    }
    ++shootAttempts;
    return Status::Running;
}
/// If we chip, this function is called
Status Shoot::onUpdateChip(const StpInfo &info) noexcept {
    // Clamp and set chip velocity
    float chipVelocity = std::clamp(info.getKickChipVelocity(), 0.0, stp::control_constants::MAX_CHIP_POWER);

    // Set chip command
    command.kickType = KickType::CHIP;
    command.kickSpeed = chipVelocity;

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    int targetDribblerSpeed = static_cast<int>(targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD);

    // Set dribbler speed command
    command.dribblerSpeed = targetDribblerSpeed;

    // Set angle command
    command.targetAngle = info.getRobot().value()->getAngle();

    // Set chip_kick_forced if we can chip but did not chip for MAX_CHIP_ATTEMPTS amount of ticks
    if (shootAttempts > control_constants::MAX_CHIP_ATTEMPTS) {
        command.waitForBall = false;
        shootAttempts = 0;
    } else {
        command.waitForBall = true;
    }
    // set command ID
    command.id = info.getRobot().value()->getId();

    // publish the generated command
    forwardRobotCommand(info.getCurrentWorld());

    if (!info.getRobot().value()->hasBall()) {
        shootAttempts = 0;
        return Status::Success;
    }
    ++shootAttempts;
    return Status::Running;
}

const char *Shoot::getName() { return "Shoot"; }

}  // namespace rtt::ai::stp::skill
