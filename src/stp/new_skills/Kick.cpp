//
// Created by jordi on 03-03-20.
//

#include "stp/new_skills/Kick.h"

namespace rtt::ai::stp::skill {

void Kick::onInitialize() noexcept {}

Status Kick::onUpdate(const StpInfo &info) noexcept {
    // Clamp and set kick velocity
    double kickVelocity = std::clamp(info.getKickChipVelocity(), 0.0, stp::control_constants::MAX_KICK_POWER);

    // Set kick command
    command.set_kicker(true);

    command.set_chip_kick_vel(kickVelocity);

    command.set_dribbler(info.getDribblerSpeed() / 100 * 32);

    // Set angle command
    command.set_w(info.getRobot().value()->getAngle());

    publishRobotCommand(info.getCurrentWorld());

    if (info.getBall()->get()->getVelocity().length() > stp::control_constants::HAS_KICKED_ERROR_MARGIN &&
        info.getRobot().value()->getAngleDiffToBall() <= control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI) {
        return Status::Success;
    }
    return Status::Running;
}

void Kick::onTerminate() noexcept {}

const char *Kick::getName() { return "Kick"; }

}  // namespace rtt::ai::stp::skill