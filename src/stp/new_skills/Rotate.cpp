//
// Created by jordi on 06-03-20.
//

#include "stp/new_skills/Rotate.h"

#include <roboteam_utils/Print.h>
#include <include/roboteam_ai/control/ControlUtils.h>

namespace rtt::ai::stp::skill {

void Rotate::onInitialize() noexcept {}

Status Rotate::onUpdate(const StpInfo &info) noexcept {
    // Constrain and set angle between -pi and pi
    double targetAngle = rtt::ai::control::ControlUtils::constrainAngleMinusPiToPi(info.getAngle());

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    int targetDribblerSpeed = targetDribblerPercentage / 100.0 * Constants::MAX_DRIBBLER_CMD();

    // Set angle command
    command.set_w(targetAngle);

    // Set dribbler speed command
    command.set_dribbler(targetDribblerSpeed);

    publishRobotCommand();

    // Check if successful
    double errorMargin = Constants::GOTOPOS_ANGLE_ERROR_MARGIN() * M_PI;
    if (rtt::ai::control::ControlUtils::angleDifference(info.getRobot().value()->getAngle().getAngle(), targetAngle) < errorMargin) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

void Rotate::onTerminate() noexcept {}

}  // namespace rtt::ai::stp::skill
