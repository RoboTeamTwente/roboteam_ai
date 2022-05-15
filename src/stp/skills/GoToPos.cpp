//
// Created by jordi on 09-03-20.
//

#include "stp/skills/GoToPos.h"

#include "roboteam_utils/Print.h"
#include "roboteam_utils/Timer.h"
#include "world/World.hpp"

namespace rtt::ai::stp::skill {

Status GoToPos::onUpdate(const StpInfo &info) noexcept {
    auto &robotValue = robot.value();
    Vector2 targetPos = info.getPositionToMoveTo().value();

    if (!targetPos.isNotNaN()) {
        RTT_ERROR("GoToPos: targetPos is nan for robot id: ", robotValue->getId());
        return Status::Failure;
    }

    if (!FieldComputations::pointIsValidPosition(info.getField().value(), targetPos, info.getObjectsToAvoid())) {
        RTT_WARNING("Target point is not a valid position for robot id: ", info.getRobot().value()->getId())
        targetPos = FieldComputations::projectPointToValidPosition(info.getField().value(), targetPos, info.getObjectsToAvoid());
    }

    auto controlCommand =
        info.getCurrentWorld()->getRobotPositionController()->computeAndTrackTrajectory(info.getField().value(), robotValue->getId(), robotValue->getPos(), robotValue->getVel(),
                                                                                        targetPos, info.getMaxRobotVelocity(), info.getPidType().value(), info.getObjectsToAvoid());

    if (controlCommand.isOccupied) {
        // TODO: This should NOT be neccessary
        command.velocity = Vector2(0, 0);
        forwardRobotCommand(info.getCurrentWorld());
        return Status::Failure;
    }

    double targetVelocityLength;
    if (info.getPidType() == stp::PIDType::KEEPER && (info.getRobot()->get()->getPos() - info.getBall()->get()->getPos()).length() > control_constants::ROBOT_RADIUS / 2) {
        RTT_DEBUG("Setting max vel");
        targetVelocityLength = info.getMaxRobotVelocity();
    } else {
        targetVelocityLength = std::clamp(controlCommand.robotCommand.velocity.length(), 0.0, info.getMaxRobotVelocity());
    }
    Vector2 targetVelocity = controlCommand.robotCommand.velocity.stretchToLength(targetVelocityLength);

    // Set velocity and angle commands
    command.velocity = targetVelocity;

    command.targetAngle = info.getAngle();

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    double targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

    // Set dribbler speed command
    command.dribblerSpeed = targetDribblerSpeed;

    // set command ID
    //    command.id = robot.value()->getId();

    // forward the generated command to the ControlModule, for checking and limiting
    forwardRobotCommand(info.getCurrentWorld());

    // Check if successful
    if ((robotValue->getPos() - targetPos).length() <= stp::control_constants::GO_TO_POS_ERROR_MARGIN) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

const char *GoToPos::getName() { return "Go To Position"; }

}  // namespace rtt::ai::stp::skill