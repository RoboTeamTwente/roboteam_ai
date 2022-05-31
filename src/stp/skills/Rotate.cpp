//
// Created by jordi on 06-03-20.
//

#include "stp/skills/Rotate.h"

#include "control/ControlUtils.h"
#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::skill {

Status Rotate::onUpdate(const StpInfo &info) noexcept {
    auto targetAngle = info.getAngle();

    // Set angle command
    command.targetAngle = targetAngle;

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    double targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

    // Set dribbler speed command
    command.dribblerSpeed = targetDribblerSpeed;

    // set command ID
    command.id = info.getRobot().value()->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    // Check if the robot is within the error margin
    double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    if (info.getRobot().value()->getAngle().shortestAngleDiff(targetAngle) < errorMargin) {
        withinMarginCount += 1;
    } else {
        withinMarginCount = 0;
    }

    // Check whether the robot has been within the margin
    if (withinMarginCount > 5) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *Rotate::getName() { return "Rotate"; }

}  // namespace rtt::ai::stp::skill
