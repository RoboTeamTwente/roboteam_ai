//
// Created by jordi on 09-03-20.
//

#include "include/roboteam_ai/stp/new_skills/Chip.h"

namespace rtt::ai::stp::skill {

void Chip::onInitialize() noexcept {}

Status Chip::onUpdate(const StpInfo &info) noexcept {
    // Clamp and set chip velocity
    double chipVelocity = std::clamp(info.getKickChipVelocity(), 0.0, Constants::MAX_KICK_POWER());

    // Set chip command
    command.set_chipper(true);
    command.set_chip_kick_vel(chipVelocity);

    // Set angle command
    command.set_w(info.getRobot().value()->getAngle().getAngle());

    publishRobotCommand();

    if (info.getBall()->get()->getVelocity().length() > 0.6) {
        return Status::Success;
    }
    return Status::Running;
}

void Chip::onTerminate() noexcept {}

}  // namespace rtt::ai::stp::skill
