//
// Created by jordi on 09-03-20.
//

#include "stp/new_skills/Chip.h"

namespace rtt::ai::stp::skill {

void Chip::onInitialize() noexcept {}

Status Chip::onUpdate(const StpInfo &info) noexcept {
    // Clamp and set chip velocity
    double chipVelocity = std::clamp(info.getKickChipVelocity(), 0.0, stp::control_constants::MAX_KICK_POWER);

    // Set chip command
    command.set_chipper(true);
    command.set_chip_kick_vel(chipVelocity);

    // Set angle command
    command.set_w(info.getRobot().value()->getAngle().getAngle());

    publishRobotCommand();

    if (info.getBall()->get()->getVelocity().length() > stp::control_constants::BALL_IS_MOVING_VEL) {
        return Status::Success;
    }
    return Status::Running;
}

void Chip::onTerminate() noexcept {}

}  // namespace rtt::ai::stp::skill
