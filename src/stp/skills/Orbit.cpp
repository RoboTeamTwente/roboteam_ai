//
// Created by mamiksik on 22-04-21.
//

#include "stp/skills/Orbit.h"

#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::skill {

Status Orbit::onUpdate(const StpInfo &info) noexcept {
    Vector2 directionVector = (info.getBall()->get()->position - info.getRobot()->get()->getPos());
    double normalAngle = directionVector.rotate(M_PI).rotate(M_PI_2).angle();
    Angle targetAngle = (info.getPositionToShootAt().value() - info.getBall()->get()->position).toAngle();

    double margin = control_constants::ROBOT_RADIUS + 1.5 * stp::control_constants::BALL_RADIUS;
    double adjustDistance = (info.getRobot()->get()->getDistanceToBall() - margin);

    // Get the direction of movement, counterclockwise or clockwise
    auto direction = Angle(directionVector).rotateDirection(targetAngle) ? -1.0 : 1.0;

    double error = targetAngle.shortestAngleDiff(directionVector.angle());
    error = error * direction;

    // Use PID controller to determine desired velocity multiplier
    auto multiplier = velPid.getOutput(error, 0);

    // Velocity consists of a part to create the circular movement, and an adjustment for distance to the ball
    Vector2 targetVelocity;
    targetVelocity.x = cos(normalAngle) * multiplier + 3 * cos(directionVector.toAngle()) * adjustDistance;
    targetVelocity.y = sin(normalAngle) * multiplier + 3 * sin(directionVector.toAngle()) * adjustDistance;

    auto maxVel = directionVector.length() * 4;
    if (maxVel < 0.65) maxVel = 0.65;
    if (targetVelocity.length() > maxVel) targetVelocity = targetVelocity.stretchToLength(maxVel);

    command.velocity = targetVelocity * 0.5;

    command.targetAngle = targetAngle;

    // set command ID
    command.id = info.getRobot().value()->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    // Check if successful
    double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    if (directionVector.toAngle().shortestAngleDiff(targetAngle) < errorMargin) {
        counter++;
    } else {
        counter = 0;
    }

    // If the robot is within the error margin for 5 consecutive ticks, return success
    if (counter > 5)
        return Status::Success;
    else
        return Status::Running;
}

const char *Orbit::getName() { return "Orbit"; }

}  // namespace rtt::ai::stp::skill