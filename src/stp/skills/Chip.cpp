//
// Created by jordi on 09-03-20.
// used in ChipAtPos tactic

#include "stp/skills/Chip.h"

#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::skill {

Status Chip::onUpdate(const StpInfo &info) noexcept {
    // Clamp and set chip velocity
    float chipVelocity = std::clamp(info.getKickChipVelocity(), 0.0, stp::control_constants::MAX_CHIP_POWER);

    // Set chip command
    command.kickType = KickType::CHIP;
    command.kickSpeed = chipVelocity;

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 10);
    double targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

    // Set dribbler speed command
    command.dribblerSpeed = targetDribblerSpeed;

    // Set angle command
    command.targetAngle = info.getRobot().value()->getAngle();

    // Set chip_kick_forced if we can chip but did not chip for MAX_CHIP_ATTEMPTS amount of ticks
    if (chipAttempts > control_constants::MAX_CHIP_ATTEMPTS) {
        command.waitForBall = false;
        chipAttempts = 0;
    } else {
        command.waitForBall = true;
    }

    // set command ID
    command.id = info.getRobot().value()->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    if (info.getBall()->get()->velocity.length() > stp::control_constants::HAS_CHIPPED_ERROR_MARGIN) {
        chipAttempts = 0;
        return Status::Success;
    }
    ++chipAttempts;
    return Status::Running;
}

const char *Chip::getName() { return "Chip"; }

}  // namespace rtt::ai::stp::skill
