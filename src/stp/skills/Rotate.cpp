//
// Created by jordi on 06-03-20.
//

#include "stp/skills/Rotate.h"

#include "control/ControlUtils.h"

namespace rtt::ai::stp::skill {

Status Rotate::onUpdate(const StpInfo &info) noexcept {
    auto targetAngle = info.getAngle();

    // Set angle command
    command.set_w(static_cast<float>(targetAngle));

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 30);
    int targetDribblerSpeed = static_cast<int>(targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD);

    // Set dribbler speed command
    command.set_dribbler(targetDribblerSpeed);

    // set command ID
    command.set_id(info.getRobot().value()->getId());

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    // Check if successful
    double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    if (info.getRobot().value()->getAngle().shortestAngleDiff(targetAngle) < errorMargin) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *Rotate::getName() { return "Rotate"; }

}  // namespace rtt::ai::stp::skill
