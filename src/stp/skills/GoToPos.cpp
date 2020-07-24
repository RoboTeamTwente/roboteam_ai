//
// Created by jordi on 09-03-20.
//

#include "stp/skills/GoToPos.h"

#include "world_new/World.hpp"

namespace rtt::ai::stp::skill {

Status GoToPos::onUpdate(const StpInfo &info) noexcept {
    Vector2 targetPos = info.getPositionToMoveTo().value();

    if (!FieldComputations::pointIsInField(info.getField().value(), targetPos)) {
        RTT_WARNING("Target point not in field for robot ID ", info.getRobot().value()->getId());
        targetPos = control::ControlUtils::projectPositionToWithinField(info.getField().value(), targetPos, control_constants::ROBOT_RADIUS);
    }

    // Calculate commands from path planning and tracking
    auto robotCommand = info.getCurrentWorld()->getRobotPositionController()->computeAndTrackPath(
        info.getField().value(), info.getRobot().value()->getId(), info.getRobot().value()->getPos(), info.getRobot().value()->getVel(), targetPos, info.getPidType().value());

    // Clamp and set velocity
    double targetVelocityLength = std::clamp(robotCommand.vel.length(), 0.0, stp::control_constants::MAX_VEL_CMD);
    Vector2 targetVelocity = robotCommand.vel.stretchToLength(targetVelocityLength);

    // Set velocity and angle commands
    command.mutable_vel()->set_x(static_cast<float>(targetVelocity.x));
    command.mutable_vel()->set_y(static_cast<float>(targetVelocity.y));

    command.set_w(info.getAngle());

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    int targetDribblerSpeed = static_cast<int>(targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD);

    // Set dribbler speed command
    command.set_dribbler(targetDribblerSpeed);

    // publish the generated command
    publishRobotCommand(info.getCurrentWorld());

    // Check if successful
    if ((info.getRobot().value()->getPos() - targetPos).length() <= stp::control_constants::GO_TO_POS_ERROR_MARGIN) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *GoToPos::getName() { return "Go To Position"; }

}  // namespace rtt::ai::stp::skill