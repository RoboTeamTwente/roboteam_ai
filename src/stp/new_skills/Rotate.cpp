//
// Created by jordi on 06-03-20.
//

#include "include/roboteam_ai/stp/new_skills/Rotate.h"

namespace rtt::ai::stp {

Status Rotate::onInitialize() noexcept {
    return Status::Success;
}

Status Rotate::onUpdate(const rtt::ai::stp::SkillInfo &info) noexcept {
    double angle = info.getAngle();
    int dribblerSpeed = info.getDribblerSpeed();

    // Check if angle is in range
    if (angle < -M_PI || angle > M_PI) {
        return Status::Failure;
    }

    // Check if dribbler speed is in range
    if (dribblerSpeed < 0 || dribblerSpeed > 31) {
        return Status::Failure;
    }

    // Set angle command
    command.set_w(angle);

    // Set dribbler speed command
    command.set_dribbler(dribblerSpeed);

    publishRobotCommand();

    return Status::Success;
}

Status Rotate::onTerminate() noexcept {
    return Status::Success;
}

} // namespace rtt::ai::stp