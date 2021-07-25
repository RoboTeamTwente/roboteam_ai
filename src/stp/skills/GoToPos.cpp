//
// Created by jordi on 09-03-20.
//

#include "stp/skills/GoToPos.h"

#include "world/World.hpp"

namespace rtt::ai::stp::skill {

Status GoToPos::onUpdate(const StpInfo &info) noexcept {
    Vector2 targetPos = info.getPositionToMoveTo().value();

    if (!FieldComputations::pointIsInField(info.getField().value(), targetPos)) {
        RTT_WARNING("Target point not in field for robot ID ", info.getRobot().value()->getId())
        targetPos = control::ControlUtils::projectPositionToWithinField(info.getField().value(), targetPos, control_constants::ROBOT_RADIUS);
    }

    bool useOldPathPlanning = true;
    rtt::BB::CommandCollision commandCollision;

    if(useOldPathPlanning) {
        // Calculate commands from path planning and tracking
        commandCollision.robotCommand = info.getCurrentWorld()->getRobotPositionController()->computeAndTrackPath(
            info.getField().value(), info.getRobot().value()->getId(), info.getRobot().value()->getPos(), info.getRobot().value()->getVel(), targetPos, info.getPidType().value());
    } else {
        // _______Use this one for the BBT pathplanning and tracking_______
        commandCollision = info.getCurrentWorld()->getRobotPositionController()->computeAndTrackPathBBT(
            info.getCurrentWorld(), info.getField().value(), info.getRobot().value()->getId(), info.getRobot().value()->getPos(),
            info.getRobot().value()->getVel(), targetPos, info.getPidType().value());
    }

    if (commandCollision.collisionData.has_value()) {
        return Status::Failure;
    }
    // Clamp and set velocity
    double targetVelocityLength = std::clamp(commandCollision.robotCommand.vel.length(), 0.0, stp::control_constants::MAX_VEL_CMD);
    Vector2 targetVelocity = commandCollision.robotCommand.vel.stretchToLength(targetVelocityLength);

    // Set velocity and angle commands
    command.mutable_vel()->set_x(static_cast<float>(targetVelocity.x));
    command.mutable_vel()->set_y(static_cast<float>(targetVelocity.y));

    command.set_w(static_cast<float>(info.getAngle()));

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    int targetDribblerSpeed = static_cast<int>(targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD);

    // Set dribbler speed command
    command.set_dribbler(targetDribblerSpeed);

    // set command ID
    command.set_id(info.getRobot().value()->getId());

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    // Check if successful
    if ((info.getRobot().value()->getPos() - targetPos).length() <= stp::control_constants::GO_TO_POS_ERROR_MARGIN) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *GoToPos::getName() { return "Go To Position"; }

}  // namespace rtt::ai::stp::skill