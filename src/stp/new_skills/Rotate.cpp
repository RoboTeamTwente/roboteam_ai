//
// Created by jordi on 06-03-20.
//

#include "include/roboteam_ai/stp/new_skills/Rotate.h"

#include <roboteam_utils/Print.h>

namespace rtt::ai::stp {

void Rotate::onInitialize() noexcept {}

Status Rotate::onUpdate(const rtt::ai::stp::SkillInfo &info) noexcept {
    robot = info.getRobot();

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
    if (robot.value()->getAngle() == targetAngle + errorMargin && robot.value()->getDribblerState() == targetDribblerSpeed) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

void Rotate::onTerminate() noexcept {}

}  // namespace rtt::ai::stp
