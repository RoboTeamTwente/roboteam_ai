//
// Created by jordi on 09-03-20.
//

#include "include/roboteam_ai/stp/new_skills/GoToPos.h"

#include <roboteam_utils/Print.h>

#include "include/roboteam_ai/world_new/World.hpp"

namespace rtt::ai::stp::skill {

void GoToPos::onInitialize() noexcept { }

Status GoToPos::onUpdate(const StpInfo &info) noexcept {
    Vector2 targetPos = info.getTargetPos().second;

    // Calculate commands from path planning and tracking
    auto robotCommand = world_new::World::instance()->getRobotPositionController()->
            computeAndTrackPath(info.getField().value(), info.getRobot().value()->getId(),
                    info.getRobot().value()->getPos(), info.getRobot().value()->getVel(), targetPos);

    // Clamp and set velocity
    double targetVelocityLength = std::clamp(robotCommand.vel.length(), 0.0, Constants::MAX_VEL_CMD());
    Vector2 targetVelocity = robotCommand.vel.stretchToLength(targetVelocityLength);

    // Set velocity and angle commands
    command.mutable_vel()->set_x(targetVelocity.x);
    command.mutable_vel()->set_y(targetVelocity.y);
    command.set_w(robotCommand.angle.getAngle());

    // Clamp and set dribbler speed
    int targetDribblerPercentage = std::clamp(info.getDribblerSpeed(), 0, 100);
    int targetDribblerSpeed = targetDribblerPercentage / 100.0 * Constants::MAX_DRIBBLER_CMD();

    // Set dribbler speed command
    command.set_dribbler(targetDribblerSpeed);

    publishRobotCommand();

    // Check if successful
    double errorMargin = Constants::GOTOPOS_ERROR_MARGIN();
    if ((info.getRobot().value()->getPos() - targetPos).length2() <= errorMargin * errorMargin) {
        return Status::Success;
    } else {
        return Status::Running;
    }
}

void GoToPos::onTerminate() noexcept {}

}  // namespace rtt::ai::stp::skill