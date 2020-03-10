//
// Created by jordi on 09-03-20.
//

#include "include/roboteam_ai/stp/new_skills/Chip.h"

namespace rtt::ai::stp {

void Chip::onInitialize() noexcept {}

Status Chip::onUpdate(const rtt::ai::stp::SkillInfo &info) noexcept {
    double chipVelocity = info.getKickChipVelocity();

    // Check if chip velocity is in range
    if (chipVelocity < 0.0 || chipVelocity > Constants::MAX_KICK_POWER()) {
        return Status::Failure;
    }

    // Set chip command
    command.set_chipper(true);
    command.set_chip_kick_vel(chipVelocity);

    // Set angle command
    command.set_w(info.getRobot().value()->getAngle());

    publishRobotCommand();

    return Status::Success;
}

void Chip::onTerminate() noexcept {}

}  // namespace rtt::ai::stp
