//
// Created by jordi on 06-03-20.
//

#include "include/roboteam_ai/stp/new_skills/Rotate.h"
#include <roboteam_utils/Print.h>

namespace rtt::ai::stp {

void Rotate::onInitialize() noexcept { }

Status Rotate::onUpdate(const rtt::ai::stp::SkillInfo &info) noexcept {
    float angle = info.getAngle();
    int dribblerSpeed = info.getDribblerSpeed();

    // Check if angle is in range
    if (angle < -M_PI || angle > M_PI) {
        RTT_ERROR("Rotation angle not within acceptable range")
        return Status::Failure;
    }

    // Check if dribbler speed is in range
    if (dribblerSpeed < 0 || dribblerSpeed > 31) {
        RTT_ERROR("Dribbler speed not within acceptable range")
        return Status::Failure;
    }

    // Set angle command
    command.set_w(angle);

    // Set dribbler speed command
    command.set_dribbler(dribblerSpeed);

    publishRobotCommand();

    // Check if successful
    double errorMargin = 0.03 * M_PI;
    if (info.getRobot()->getAngle().shortestAngleDiff(angle) <= errorMargin) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

void Rotate::onTerminate() noexcept { }

} // namespace rtt::ai::stp
