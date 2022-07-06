//
// Created by jordi on 03-03-20.
//

#include "stp/skills/Kick.h"

#include "stp/constants/ControlConstants.h"
#include "roboteam_utils/Print.h"

namespace rtt::ai::stp::skill {

Status Kick::onUpdate(const StpInfo &info) noexcept {
    // Clamp and set kick velocity
    float kickVelocity = std::clamp(info.getKickChipVelocity(), 0.0, stp::control_constants::MAX_KICK_POWER);

    //RTT_ERROR("Angle in kick = ", info.getRobot()->get()->getAngle().toVector2());
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

    // Set chip_kick_forced if we can chip but did not chip for MAX_CHIP_ATTEMPTS amount of ticks
    if (kickAttempts > control_constants::MAX_KICK_ATTEMPTS) {
        command.waitForBall = false;
        kickAttempts = 0;
    } else {
        command.waitForBall = false;  // Apparently, waiting for the ball is the default
    }

    // set command ID
    command.id = info.getRobot().value()->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    if (!info.getRobot().value()->hasBall()) {
        kickAttempts = 0;
        return Status::Running;
    }
    ++kickAttempts;
    return Status::Running;
}

const char *Kick::getName() { return "Kick"; }

}  // namespace rtt::ai::stp::skill