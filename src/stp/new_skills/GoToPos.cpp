//
// Created by jordi on 09-03-20.
//

#include "include/roboteam_ai/stp/new_skills/GoToPos.h"

#include <roboteam_utils/Print.h>

#include "include/roboteam_ai/world_new/World.hpp"

namespace rtt::ai::stp {

void GoToPos::onInitialize() noexcept {
    RTT_WARNING("INITIALIZING GOTOPOS");
}

Status GoToPos::onUpdate(const StpInfo &info) noexcept {
    RTT_WARNING("UPDATING GOTOPOS")
    Vector2 targetPos = info.getTargetPos().second;

    // Calculate commands from path planning and tracking
    auto robotCommand = world_new::World::instance()->getRobotPositionController()->computeAndTrackPath(
        info.getField().value(), info.getRobot().value()->getId(), info.getRobot().value()->getPos(), info.getRobot().value()->getVel(), targetPos);

    // Check if velocity is in range
    //TODo clamp this
    if (robotCommand.vel.length() > Constants::MAX_VEL_CMD()) {
        RTT_ERROR("Velocity not within acceptable range")
        //return Status::Failure;
    }

    // Check if angle is in range
    //TODo clamp this
    if (robotCommand.angle.getAngle() < Constants::MIN_ANGLE() || robotCommand.angle.getAngle() > Constants::MAX_ANGLE()) {
        RTT_ERROR("Rotation angle not within acceptable range")
        //return Status::Failure;
    }

    // Set velocity and angle commands
    command.mutable_vel()->set_x(robotCommand.vel.x);
    command.mutable_vel()->set_y(robotCommand.vel.y);
    command.set_w(robotCommand.angle);

    // Check if dribbler speed is in range
    int dribblerSpeed = info.getDribblerSpeed();
    if (dribblerSpeed < 0 || dribblerSpeed > 31) {
        RTT_ERROR("Dribbler speed not within acceptable range")
        return Status::Failure;
    }

    // Set dribbler speed command
    command.set_dribbler(dribblerSpeed);

    publishRobotCommand();

    // Check if successful
    double errorMargin = Constants::GOTOPOS_ERROR_MARGIN();
    if ((info.getRobot().value()->getPos() - targetPos).length2() <= errorMargin * errorMargin) {
        RTT_SUCCESS("GTP SUCCESFUL")
        return Status::Success;
    } else {
        return Status::Running;
    }
}

void GoToPos::onTerminate() noexcept {}

}  // namespace rtt::ai::stp