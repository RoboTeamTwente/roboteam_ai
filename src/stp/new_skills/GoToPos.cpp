//
// Created by jordi on 09-03-20.
//

#include "include/roboteam_ai/stp/new_skills/GoToPos.h"


#include "include/roboteam_ai/world_new/World.hpp"

namespace rtt::ai::stp::skill {

void GoToPos::onInitialize() noexcept {}

Status GoToPos::onUpdate(const StpInfo &info) noexcept {
    auto targetPosOpt = info.getPositionToMoveTo();

    if (!targetPosOpt) {
        return Status::Running;
    }

    auto targetPos = targetPosOpt.value();

    // Calculate commands from path planning and tracking
    auto const& [_, world] = world_new::World::instance();
    auto robotCommand = world->getRobotPositionController()->computeAndTrackPath(
        info.getField().value(), info.getRobot().value()->getId(), info.getRobot().value()->getPos(), info.getRobot().value()->getVel(), targetPos);

    // Clamp and set velocity
    double targetVelocityLength = std::clamp(robotCommand.vel.length(), 0.0, stp::control_constants::MAX_VEL_CMD);
    Vector2 targetVelocity = robotCommand.vel.stretchToLength(targetVelocityLength);

    // Set velocity and angle commands
    command.mutable_vel()->set_x(targetVelocity.x);
    command.mutable_vel()->set_y(targetVelocity.y);

    if(info.getAngle()) command.set_w(info.getAngle());
    else command.set_w(robotCommand.angle);

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    int targetDribblerSpeed = targetDribblerPercentage / 100.0 * stp::control_constants::MAX_DRIBBLER_CMD;

    // Set dribbler speed command
    command.set_dribbler(targetDribblerSpeed);

    publishRobotCommand();

    // Check if successful
    if ((info.getRobot().value()->getPos() - targetPos).length() <= stp::control_constants::GO_TO_POS_ERROR_MARGIN || info.getRobot().value().hasBall()) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

void GoToPos::onTerminate() noexcept {}

const char *GoToPos::getName() {
    return "Go To Position";
}

}  // namespace rtt::ai::stp::skill