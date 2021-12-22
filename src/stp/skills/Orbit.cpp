//
// Created by mamiksik on 22-04-21.
//

#include "stp/skills/Orbit.h"

#include <ApplicationManager.h>

#include "control/ControlUtils.h"

namespace rtt::ai::stp::skill {

Status Orbit::onUpdate(const StpInfo &info) noexcept {
    Vector2 directionVector = (info.getBall()->get()->getPos() - info.getRobot()->get()->getPos());
    Angle normalAngle = directionVector.rotate(M_PI).rotate(M_PI_2).toAngle();

    Vector2 initialVelocity = info.getRobot()->get()->getVel();
    double initX = cos(initialVelocity.toAngle()) * initialVelocity.length();
    double initY = sin(initialVelocity.toAngle()) * initialVelocity.length();

    double margin = stp::control_constants::ROBOT_RADIUS + 2 * stp::control_constants::BALL_RADIUS;
    Vector2 adjustDistance = directionVector.normalize().scale(info.getRobot()->get()->getDistanceToBall() - margin);

    double multiplier = -sin(0.5 * (directionVector - info.getAngle()).toAngle());

    double x = cos(normalAngle) * multiplier + cos(adjustDistance.toAngle()) * adjustDistance.length();  //- initX adjustment for initial velocity?;
    double y = sin(normalAngle) * multiplier + sin(adjustDistance.toAngle()) * adjustDistance.length();  //- initY adjustment for initial velocity?;

    double errorMargin = stp::control_constants::GO_TO_POS_ANGLE_ERROR_MARGIN * M_PI;
    command.set_w(info.getAngle());

    // Don't rotate and move at the same time
    if (info.getRobot().value()->getAngle().shortestAngleDiff(info.getAngle()) < M_PI_4) {
        command.mutable_vel()->set_x(static_cast<float>(x));
        command.mutable_vel()->set_y(static_cast<float>(y));
    }

    // set command ID
    command.set_id(info.getRobot().value()->getId());

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    // Check if successful
    if (directionVector.rotate(M_PI).toAngle().shortestAngleDiff(info.getAngle()) < errorMargin) {
        return Status::Success;
    }

    return Status::Running;
}

const char *Orbit::getName() { return "Orbit"; }

}  // namespace rtt::ai::stp::skill