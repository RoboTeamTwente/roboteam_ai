//
// Created by tijmen on 01-07-22.
//

#include "stp/skills/OrbitAngular.h"

#include "stp/constants/ControlConstants.h"
#include <roboteam_utils/Print.h>

namespace rtt::ai::stp::skill {

Status OrbitAngular::onUpdate(const StpInfo &info) noexcept {
    Vector2 directionVector = info.getRobot()->get()->getAngle().toVector2();
    Vector2 normalVector = directionVector.rotate(M_PI_2);
    Angle targetAngle = (info.getPositionToShootAt().value() - info.getBall()->get()->position).toAngle();

    // Get the direction of movement, counterclockwise or clockwise
    auto direction = Angle(directionVector).rotateDirection(targetAngle) ? 1.0 : -1.0;
    //normalVector = normalVector*direction;

    // Let robot move in direction normal to the direction of the robot (sideways)
    Vector2 targetVelocity;
    targetVelocity.x = cos(normalVector.angle());
    targetVelocity.y = sin(normalVector.angle());

    command.velocity = targetVelocity.stretchToLength(0.1);

    command.useAngularVelocity = true;
    command.targetAngularVelocity = -1;
    //RTT_DEBUG("Orbit set angular velocity: ", command.targetAngularVelocity);

    // set command ID
    command.id = info.getRobot().value()->getId();
    command.dribblerSpeed = info.getDribblerSpeed();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    // Check if successful
    double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    if (directionVector.toAngle().shortestAngleDiff(targetAngle) < errorMargin) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *OrbitAngular::getName() { return "OrbitAngular"; }

}  // namespace rtt::ai::stp::skill