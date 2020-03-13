//
// Created by jordi on 03-03-20.
//

#include "include/roboteam_ai/stp/new_skills/Kick.h"

namespace rtt::ai::stp::skill {

void Kick::onInitialize() noexcept {}

Status Kick::onUpdate(const StpInfo &info) noexcept {
    // Clamp and set kick velocity
    double kickVelocity = std::clamp(info.getKickChipVelocity(), 0.0, Constants::MAX_KICK_POWER());

    // Set kick command
    command.set_kicker(true);
    command.set_chip_kick_vel(kickVelocity);

    // Set angle command
    command.set_w(info.getRobot().value()->getAngle().getAngle());

    publishRobotCommand();

    return Status::Success;
}

void Kick::onTerminate() noexcept {}

}  // namespace rtt::ai::stp::skill