//
// Created by tijmen on 01-07-22.
//

#include "stp/skills/OrbitAngular.h"

#include "stp/constants/ControlConstants.h"
#include "utilities/Settings.h"

namespace rtt::ai::stp::skill {

// Constants used to tweak the orbit to be a nice circle
constexpr double ANGLE_RATE_FACTOR = 1.0;
constexpr double VELOCITY_FACTOR = 1.2;
constexpr double RPM = 20; // Number of orbits (rotations) per minute
constexpr double ANGULAR_VELOCITY = -RPM/60*2*M_PI * ANGLE_RATE_FACTOR;

Status OrbitAngular::onUpdate(const StpInfo &info) noexcept {

    double circumference = (control_constants::ROBOT_RADIUS + stp::control_constants::BALL_RADIUS) * 2*M_PI;
    double rotVel = RPM/60*circumference * VELOCITY_FACTOR;

    Vector2 directionVector = info.getRobot()->get()->getAngle().toVector2();
    Angle targetAngle = (info.getPositionToShootAt().value() - info.getRobot()->get()->getPos()).toAngle();

    auto direction = Angle(directionVector).rotateDirection(targetAngle) ? -1.0 : 1.0;

    if (SETTINGS.getRobotHubMode() == Settings::RobotHubMode::BASESTATION) {
        // Since there is a small delay between sending kick and the ball being kicked, we want to adjust the targetAngle so that we kick slightly earlier
        targetAngle = targetAngle.getValue() - ANGULAR_VELOCITY / 20.0 * direction;
        direction = Angle(directionVector).rotateDirection(targetAngle) ? -1.0 : 1.0;
    }

    Vector2 normalVector = directionVector.rotate(direction * (M_PI_2 - (1.0/3.0)*M_PI));

    // Let robot move in direction normal to the direction of the robot (sideways)
    Vector2 targetVelocity;
    targetVelocity.x = normalVector.x;
    targetVelocity.y = normalVector.y;

    command.velocity = targetVelocity.stretchToLength(rotVel);

    command.useAngularVelocity = true;
    command.targetAngularVelocity = ANGULAR_VELOCITY*direction;
    //RTT_DEBUG("Orbit set angular velocity: ", command.targetAngularVelocity);

    // set command ID
    command.id = info.getRobot().value()->getId();

    if (SETTINGS.getRobotHubMode() == Settings::RobotHubMode::BASESTATION){
        // set angle to kick at & turn on kickAtAngle
        command.kickType = KickType::KICK;
        command.kickSpeed = std::clamp(info.getKickChipVelocity(), control_constants::MIN_KICK_POWER, control_constants::MAX_KICK_POWER);
        command.targetAngle = targetAngle;
        command.kickAtAngle = true;
    } else if (info.getRobot()->get()->getAngle().shortestAngleDiff(targetAngle) < control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI) {
        command.kickSpeed = std::clamp(info.getKickChipVelocity(), control_constants::MIN_KICK_POWER, control_constants::MAX_KICK_POWER);
        command.kickType = KickType::KICK;
    }
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    double targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

    command.dribblerSpeed = targetDribblerSpeed;

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    // Check if successful- Done after the ball has been kicked (if the velocity is low, we probably did not kick it, so we didn't finish successfully)
    if (!info.getRobot()->get()->hasBall() && info.getBall()->get()->velocity.length() > control_constants::BALL_GOT_SHOT_LIMIT){
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *OrbitAngular::getName() { return "OrbitAngular"; }

}  // namespace rtt::ai::stp::skill