//
// Created by jordi on 06-03-20.
//

#include "stp/new_skills/Rotate.h"

#include <roboteam_utils/Print.h>

namespace rtt::ai::stp {

void Rotate::onInitialize() noexcept {}

Status Rotate::onUpdate(const StpInfo &info) noexcept {
    RTT_WARNING("UPDATING ROTATE")
    float targetAngle = info.getAngle();
    int targetDribblerSpeed = info.getDribblerSpeed();

    // Check if angle is in range
    if (targetAngle < -M_PI || targetAngle > M_PI) {
        RTT_ERROR("Rotation angle not within acceptable range")
        return Status::Failure;
    }

    // Check if dribbler speed is in range
    if (targetDribblerSpeed < 0 || targetDribblerSpeed > 31) {
        RTT_ERROR("Dribbler speed not within acceptable range")
        return Status::Failure;
    }

    // Set angle command
    command.set_w(targetAngle);

    // Set dribbler speed command
    command.set_dribbler(targetDribblerSpeed);

    publishRobotCommand();

    // Check if successful
    double errorMargin = 0.03 * M_PI;
    if (fabs(robot.value()->getAngle().getAngle() - targetAngle) < errorMargin) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

void Rotate::onTerminate() noexcept {}

}  // namespace rtt::ai::stp
