//
// Created by jordi on 09-03-20.
//

#include "stp/skills/Chip.h"

namespace rtt::ai::stp::skill {

Status Chip::onUpdate(const StpInfo &info) noexcept {
    // Clamp and set chip velocity
    float chipVelocity = std::clamp(info.getKickChipVelocity(), 0.0, stp::control_constants::MAX_CHIP_POWER);

    // Set chip command
    command.set_chipper(true);
    command.set_chip_kick_vel(chipVelocity);

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 10);
    int targetDribblerSpeed = static_cast<int>(targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD);

    // Set dribbler speed command
    command.set_dribbler(targetDribblerSpeed);

    // Set angle command
    command.set_w(static_cast<float>(info.getRobot().value()->getAngle()));

    // Set chip_kick_forced if we can chip but did not chip for MAX_CHIP_ATTEMPTS amount of ticks
    if (chipAttempts > control_constants::MAX_CHIP_ATTEMPTS) {
        command.set_chip_kick_forced(true);
        chipAttempts = 0;
    }
    // set command ID
    command.set_id(info.getRobot().value()->getId());

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    if (info.getBall()->get()->getVelocity().length() > stp::control_constants::HAS_CHIPPED_ERROR_MARGIN) {
        chipAttempts = 0;
        return Status::Success;
    }
    ++chipAttempts;
    return Status::Running;
}

const char *Chip::getName() { return "Chip"; }

}  // namespace rtt::ai::stp::skill
