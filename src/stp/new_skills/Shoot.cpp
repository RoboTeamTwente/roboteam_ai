//
// Created by Jesse on 23-06-20.
//

#include "stp/new_skills/Shoot.h"

#include "roboteam_utils/Print.h"

namespace rtt::ai::stp::skill {

void Shoot::onInitialize() noexcept {}

Status Shoot::onUpdate(const StpInfo &info) noexcept {
    if (info.getShootType() == rtt::ai::stp::KickChip::CHIP) {
        return onUpdateChip(info);
    } else if (info.getShootType() == rtt::ai::stp::KickChip::KICK) {
        return onUpdateKick(info);
    } else {
        RTT_ERROR("No ShootType set, kicking by default!")
        return onUpdateKick(info);
    }
}

Status Shoot::onUpdateKick(const StpInfo &info) noexcept {
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
Status Shoot::onUpdateChip(const StpInfo &info) noexcept {
    // Clamp and set chip velocity
    double chipVelocity = std::clamp(info.getKickChipVelocity(), 0.0, stp::control_constants::MAX_KICK_POWER);

    // Set chip command
    command.set_chipper(true);
    command.set_chip_kick_vel(chipVelocity);

    // Set angle command
    command.set_w(info.getRobot().value()->getAngle());

    publishRobotCommand(info.getCurrentWorld());

    if (info.getBall()->get()->getVelocity().length() > stp::control_constants::HAS_CHIPPED_ERROR_MARGIN) {
        return Status::Success;
    }
    return Status::Running;
}
void Shoot::onTerminate() noexcept {}

const char *Shoot::getName() { return "Shoot"; }

}  // namespace rtt::ai::stp::skill
