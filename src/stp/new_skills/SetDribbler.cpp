//
// Created by timovdk on 2/11/20.
//

#include "stp/new_skills/SetDribbler.h"

#include <roboteam_utils/Print.h>

namespace rtt::ai::stp::skill {

void SetDribbler::onInitialize() noexcept {}

Status SetDribbler::onUpdate(const StpInfo &info) noexcept {
    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    int targetDribblerSpeed = targetDribblerPercentage / 100.0 * Constants::MAX_DRIBBLER_CMD();

    // Set dribbler speed command
    command.set_dribbler(targetDribblerSpeed);

    // Set angle command
    command.set_w(info.getRobot().value()->getAngle().getAngle());

    publishRobotCommand();

    return Status::Running;
}

void SetDribbler::onTerminate() noexcept {}

}  // namespace rtt::ai::stp::skill
