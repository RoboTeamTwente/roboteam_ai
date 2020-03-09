//
// Created by jordi on 03-03-20.
//

#include "include/roboteam_ai/stp/new_skills/Kick.h"

namespace rtt::ai::stp {

Status Kick::onInitialize() noexcept {
    return Status::Success;
};

Status Kick::onUpdate(const rtt::ai::stp::SkillInfo &info) noexcept {
    double kickVelocity = info.getKickChipVelocity();

    // Check if kick velocity is in range
    if (kickVelocity < 0.0 || kickVelocity > Constants::MAX_KICK_POWER()) {
        return Status::Failure;
    }

    // Set kick command
    command.set_kicker(true);
    command.set_chip_kick_vel(kickVelocity);

    // Set angle command
    command.set_w(info.getRobot()->getAngle());

    publishRobotCommand();

    return Status::Success;
}

Status Kick::onTerminate() noexcept {
    return Status::Success;
}

} // namespace rtt::ai::stp