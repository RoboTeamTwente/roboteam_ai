//
// Created by tijmen on 01-07-22.
//

#include "stp/skills/OrbitAngular.h"

#include "stp/constants/ControlConstants.h"
#include <roboteam_utils/Print.h>

namespace rtt::ai::stp::skill {

Status OrbitAngular::onUpdate(const StpInfo &info) noexcept {
    double rpm = 20; // Number of orbits (rotations) per minute
    double circumference = (control_constants::ROBOT_RADIUS + stp::control_constants::BALL_RADIUS) * 2*M_PI;
    double angVel = rpm/60*2*M_PI;
    double rotVel = -rpm/60*circumference;
    RTT_DEBUG("angVel: ", angVel, " rotVel: ", rotVel);

    Vector2 directionVector = info.getRobot()->get()->getAngle().toVector2();
    Vector2 normalVector = directionVector.rotate(M_PI_2);
    Angle targetAngle = (info.getPositionToShootAt().value() - info.getBall()->get()->position).toAngle();

    if(fabs(info.getRobot()->get()->getAngularVelocity()) < 0.1) {
        // Get the direction of movement, counterclockwise or clockwise
        currentDirection = Angle(directionVector).rotateDirection(targetAngle) ? 1.0 : -1.0;
    }
    normalVector = normalVector*currentDirection;

    // Let robot move in direction normal to the direction of the robot (sideways)
    Vector2 targetVelocity;
    targetVelocity.x = cos(normalVector.angle());
    targetVelocity.y = sin(normalVector.angle());

    command.velocity = targetVelocity.stretchToLength(rotVel);

    command.useAngularVelocity = true;
    command.targetAngularVelocity = angVel*currentDirection;
    //RTT_DEBUG("Orbit set angular velocity: ", command.targetAngularVelocity);

    // set command ID
    command.id = info.getRobot().value()->getId();

    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    double targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

    // Set dribbler speed command
    command.dribblerSpeed = targetDribblerSpeed;

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    // Check if successful
    // Error margin + taking the current robotvelocity into account to kick a bit earlier
    double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI + rpm/300* M_PI;
    if (directionVector.toAngle().shortestAngleDiff(targetAngle) < errorMargin) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *OrbitAngular::getName() { return "OrbitAngular"; }

}  // namespace rtt::ai::stp::skill