//
// Created by jordi on 03-03-20.
//

#include "stp/skills/Kick.h"

#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::skill {

Status Kick::onUpdate(const StpInfo &info) noexcept {
    // Clamp and set kick velocity
    float kickVelocity = std::clamp(info.getKickChipVelocity(), control_constants::MIN_KICK_POWER, control_constants::MAX_KICK_POWER);

    // Set kick command
    command.kickType = KickType::KICK;
    command.kickSpeed = kickVelocity;

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    double targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

    // Set dribbler speed command
    command.dribblerSpeed = targetDribblerSpeed;

    // Set angle command
    command.targetAngle = info.getRobot().value()->getAngle();  // TODO: Should there be a check for robot optional?

    // TODO: test and use this code again once the ballsensor works
//    // Set chip_kick_forced if we can chip but did not chip for MAX_CHIP_ATTEMPTS amount of ticks
//    if (kickAttempts > control_constants::MAX_KICK_ATTEMPTS) {
//        command.waitForBall = false;
//        kickAttempts = 0;
//    } else {
//        command.waitForBall = true;  // Apparently, waiting for the ball is the default
//        ++kickAttempts;
//    }

    command.waitForBall = false;

    // set command ID
    command.id = info.getRobot().value()->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    if (!info.getRobot().value()->hasBall()) {
        return Status::Success;
    }
    return Status::Running;
}

const char *Kick::getName() { return "Kick"; }

}  // namespace rtt::ai::stp::skill