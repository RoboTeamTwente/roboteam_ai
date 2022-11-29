//
// Created by tijmen on 01-07-22.
//

#include "stp/skills/OrbitAngular.h"

#include "stp/constants/ControlConstants.h"
#include "roboteam_utils/Print.h"

namespace rtt::ai::stp::skill {

Status OrbitAngular::onUpdate(const StpInfo &info) noexcept {
    // initialization of local variables
    Vector2 directionVector =  info.getRobot()->get()->getAngle().toVector2();  // Vector in direction robot is currently facing
    Angle targetAngle = (info.getPositionToShootAt().value() - info.getRobot()->get()->getPos()).toAngle(); // Angle we want to have
    auto direction = Angle(directionVector).rotateDirection(targetAngle) ? 1.0 : -1.0;  // Direction robot should rotate to
    double speed = 0.1 + 4 * directionVector.toAngle().shortestAngleDiff(targetAngle); // Speed at which the robot should orbit. I made it relative to the angle so that we don't overshoot
    Vector2 normalVector = directionVector.rotate(-direction * M_PI_2); // Direction robot should move to

    // velocity vector the robot should follow
    Vector2 targetVelocity;
    targetVelocity.x = speed * normalVector.x;
    targetVelocity.y = speed * normalVector.y;

    // make robot commnad
    command.id = info.getRobot().value()->getId();
    command.velocity = targetVelocity;
    command.useAngularVelocity = true;
    command.targetAngularVelocity = direction * targetVelocity.length(); // Scale the angular velocity to the absolute velocity for a nice circle
    command.dribblerSpeed = stp::control_constants::MAX_DRIBBLER_CMD;

    // Send robot commands
    forwardRobotCommand(info.getCurrentWorld());

    // Check if successful
    double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI_2 * 0.2; // Can be finetuned. smaller margin means preciser shooting but more prone to overshooting angle
    if (directionVector.toAngle().shortestAngleDiff(targetAngle) < errorMargin) {
        counter++;
    }
    else {
        counter = 0;
    }

    // If the robot is within the error margin for 5 consecutive ticks, return success
    if (counter > 2) {
        command.dribblerSpeed = 0;
        forwardRobotCommand(info.getCurrentWorld());
        return Status::Success;
    }
    else {
        return Status::Running;
    }
}

const char *OrbitAngular::getName() { return "OrbitAngular"; }

}  // namespace rtt::ai::stp::skill