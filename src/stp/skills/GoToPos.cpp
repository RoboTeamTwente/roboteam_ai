//
// Created by jordi on 09-03-20.
//

#include "stp/skills/GoToPos.h"

#include "include/roboteam_ai/world/World.hpp"

namespace rtt::ai::stp::skill {

Status GoToPos::onUpdate(const StpInfo &info) noexcept {
    Vector2 targetPos = info.getPositionToMoveTo().value();

    if (!FieldComputations::pointIsInField(info.getField().value(), targetPos)) {
        RTT_WARNING("Target point not in field for robot ID ", info.getRobot().value()->getId())
        targetPos = control::ControlUtils::projectPositionToWithinField(info.getField().value(), targetPos, control_constants::ROBOT_RADIUS);
    }

    std::pair<RobotCommand, std::optional<Vector2>> robotCommandCollisionPair;
    // Calculate commands from path planning and tracking
    // _______Use this for the old pathplanning and tracking_______
//    robotCommandCollisionPair.first = info.getCurrentWorld()->getRobotPositionController()->computeAndTrackPath(
//        info.getField().value(), info.getRobot().value()->getId(), info.getRobot().value()->getPos(), info.getRobot().value()->getVel(), targetPos, info.getPidType().value());
    // _______Use this one for the BBT pathplanning_______
    robotCommandCollisionPair = info.getCurrentWorld()->getRobotPositionController()->computeAndTrackPathBBT(
        info.getField().value(), info.getRobot().value()->getId(), info.getRobot().value()->getPos(), info.getRobot().value()->getVel(), targetPos, info.getPidType().value());

    if (robotCommandCollisionPair.second.has_value()) {
        return Status::Failure;
    }
    // Clamp and set velocity
    double targetVelocityLength = std::clamp(robotCommandCollisionPair.first.vel.length(), 0.0, stp::control_constants::MAX_VEL_CMD);
    Vector2 targetVelocity = robotCommandCollisionPair.first.vel.stretchToLength(targetVelocityLength);

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