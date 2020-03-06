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

    if (kickVelocity < 0 || kickVelocity > Constants::MAX_KICK_POWER()) {
        return Status::Failure;
    }

    command.set_kicker(true);
    command.set_chip_kick_vel(kickVelocity);

    publishRobotCommand();

    return Status::Success;
}

Status Kick::onTerminate() noexcept {
    return Status::Success;
}

} // namespace rtt::ai::stp