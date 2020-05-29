//
// Created by jordi on 06-03-20.
//

#include "stp/new_skills/Rotate.h"

#include <include/roboteam_ai/control/ControlUtils.h>

namespace rtt::ai::stp::skill {

void Rotate::onInitialize() noexcept {}

Status Rotate::onUpdate(const StpInfo &info) noexcept {
    auto targetAngle = info.getAngle();

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    std::clamp(targetDribblerPercentage, 0, 30);
    int targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;


    // Set angle command
    command.set_w(targetAngle);

    // Set dribbler speed command
    command.set_dribbler(targetDribblerSpeed);

    publishRobotCommand();

    // Check if successful
    double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    if (info.getRobot().value()->getAngle().shortestAngleDiff(targetAngle) < errorMargin) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

void Rotate::onTerminate() noexcept {}

const char *Rotate::getName() {
    return "Rotate";
}

}  // namespace rtt::ai::stp::skill
