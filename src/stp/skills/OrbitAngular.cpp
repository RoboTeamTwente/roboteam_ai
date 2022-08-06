//
// Created by tijmen on 01-07-22.
//

#include "stp/skills/OrbitAngular.h"

#include "stp/constants/ControlConstants.h"

namespace rtt::ai::stp::skill {

Status OrbitAngular::onUpdate(const StpInfo &info) noexcept {
    double rpm = 20;  // Number of orbits (rotations) per minute
    double circumference = (control_constants::ROBOT_RADIUS + stp::control_constants::BALL_RADIUS) * 2 * M_PI;

    // Constants used to tweak the orbit to be a nice circle
    constexpr double ANGLE_RATE_FACTOR = 1.0;
    constexpr double VELOCITY_FACTOR = 1.2;

    // Calculate angVel and rotational velocity to make a circular movement around the ball
    double angVel = -rpm / 60 * 2 * M_PI * ANGLE_RATE_FACTOR;
    double rotVel = rpm / 60 * circumference * VELOCITY_FACTOR;

    Vector2 directionVector = info.getRobot()->get()->getAngle().toVector2();
    Angle targetAngle = (info.getPositionToShootAt().value() - info.getRobot()->get()->getPos()).toAngle();

    auto direction = Angle(directionVector).rotateDirection(targetAngle) ? -1.0 : 1.0;
    // Since there is a small delay between sending kick and the ball being kicked, we want to adjust the targetAngle so that we kick slightly earlier
    targetAngle = targetAngle.getValue() - angVel / 20.0 * direction;
    direction = Angle(directionVector).rotateDirection(targetAngle) ? -1.0 : 1.0;

    Vector2 normalVector = directionVector.rotate(direction * (M_PI_2 - (1.0 / 3.0) * M_PI));

    // Let robot move in direction normal to the direction of the robot (sideways)
    Vector2 targetVelocity;
    targetVelocity.x = normalVector.x;
    targetVelocity.y = normalVector.y;

    command.velocity = targetVelocity.stretchToLength(rotVel);

    command.useAngularVelocity = true;
    command.targetAngularVelocity = angVel * direction;
    // RTT_DEBUG("Orbit set angular velocity: ", command.targetAngularVelocity);

    // set command ID
    command.id = info.getRobot().value()->getId();

    // set angle to kick at & turn on kickAtAngle
    command.targetAngle = targetAngle;
    command.kickAtAngle = true;
    command.kickSpeed = std::clamp(info.getKickChipVelocity(), control_constants::MIN_KICK_POWER, control_constants::MAX_KICK_POWER);

    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    double targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

    command.dribblerSpeed = targetDribblerSpeed;

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    // Check if successful- Done after the ball has been kicked (if the velocity is low, we probably did not kick it, so we didn't finish successfully)
    if (!info.getRobot()->get()->hasBall() && info.getBall()->get()->velocity.length() > control_constants::BALL_IS_MOVING_SLOW_LIMIT) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *OrbitAngular::getName() { return "OrbitAngular"; }

}  // namespace rtt::ai::stp::skill