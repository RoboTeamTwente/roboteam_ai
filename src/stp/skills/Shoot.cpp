//
// Created by Jesse on 23-06-20.
//

#include "stp/skills/Shoot.h"

#include "roboteam_utils/Print.h"

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
    if (shootAttempts > control_constants::MAX_KICK_ATTEMPTS) {
        command.set_chip_kick_forced(true);
        shootAttempts = 0;
    }
    // set command ID
    command.set_id(info.getRobot().value()->getId());

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    if (info.getBall()->get()->getVelocity().length() > stp::control_constants::HAS_KICKED_ERROR_MARGIN) {
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
    if (shootAttempts > control_constants::MAX_CHIP_ATTEMPTS) {
        command.set_chip_kick_forced(true);
        shootAttempts = 0;
    }
    // set command ID
    command.set_id(info.getRobot().value()->getId());

    // publish the generated command
        forwardRobotCommand(info.getCurrentWorld());

    if (info.getBall()->get()->getVelocity().length() > stp::control_constants::HAS_CHIPPED_ERROR_MARGIN) {
        shootAttempts = 0;
        return Status::Success;
    }
    ++shootAttempts;
    return Status::Running;
}

const char *Shoot::getName() { return "Shoot"; }

}  // namespace rtt::ai::stp::skill
