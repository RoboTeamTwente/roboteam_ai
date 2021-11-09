//
// Created by jordi on 03-03-20.
//

#include "stp/skills/Kick.h"

namespace rtt::ai::stp::skill {

Status Kick::onUpdate(const StpInfo &info) noexcept {
    // Clamp and set kick velocity
    float kickVelocity = std::clamp(info.getKickChipVelocity(), 0.0, stp::control_constants::MAX_KICK_POWER);

    // Set kick command
    command.set_kicker(true);
    command.set_chip_kick_vel(kickVelocity);

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 10);
    int targetDribblerSpeed = static_cast<int>(targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD);

    // Set dribbler speed command
    command.set_dribbler(targetDribblerSpeed);

    // Set angle command
    command.set_w(static_cast<float>(info.getRobot().value()->getAngle()));

    // Set chip_kick_forced if we can chip but did not chip for MAX_CHIP_ATTEMPTS amount of ticks
    if (kickAttempts > control_constants::MAX_KICK_ATTEMPTS) {
        command.set_chip_kick_forced(true);
        kickAttempts = 0;
    }
    // set command ID
    command.set_id(info.getRobot().value()->getId());

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    if (info.getBall()->get()->getVelocity().length() > stp::control_constants::HAS_KICKED_ERROR_MARGIN) {
        kickAttempts = 0;
        return Status::Success;
    }
    ++kickAttempts;
    return Status::Running;
}

const char *Kick::getName() { return "Kick"; }

}  // namespace rtt::ai::stp::skill