//
// Created by timovdk on 2/11/20.
//

#include "stp/new_skills/SetDribbler.h"
#include <roboteam_utils/Print.h>

namespace rtt::ai::stp {

void SetDribbler::onInitialize() noexcept { }

Status SetDribbler::onUpdate(const rtt::ai::stp::SkillInfo &info) noexcept {
    int dribblerSpeed = info.getDribblerSpeed();

    // Check if dribbler speed is in range
    if (dribblerSpeed < 0 || dribblerSpeed > 31) {
        RTT_ERROR("Dribbler speed not within acceptable range")
        return Status::Failure;
    }

    // Set dribbler speed command
    command.set_dribbler(dribblerSpeed);

    // Set angle command
    command.set_w(info.getRobot().value()->getAngle().getAngle());

    publishRobotCommand();

    if(robot->get()->getDribblerState() == dribblerSpeed) {
        return Status::Success;
    }
    return Status::Running;
}

void SetDribbler::onTerminate() noexcept { }

}  // namespace rtt::ai
