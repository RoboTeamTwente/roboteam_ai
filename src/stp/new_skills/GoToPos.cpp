//
// Created by jordi on 09-03-20.
//

#include "include/roboteam_ai/world_new/World.hpp"
#include "include/roboteam_ai/stp/new_skills/GoToPos.h"
#include <roboteam_utils/Print.h>

namespace rtt::ai::stp {

    void GoToPos::onInitialize() noexcept { }

    Status GoToPos::onUpdate(const rtt::ai::stp::SkillInfo &info) noexcept {
        Vector2 targetPos = info.getTacticInfo().getPosition();

        // Calculate commands from path planning and tracking
        auto robotCommand = world_new::World::instance()->getRobotPositionController()->computeAndTrackPath(
                info.getTacticInfo().getField().value(), info.getRobot().value()->getId(), info.getRobot().value()->getPos(),
                info.getRobot().value()->getVel(), targetPos);

        // Check if velocity is in range
        if (robotCommand.vel.x < 0.0 || robotCommand.vel.x > Constants::MAX_VEL_CMD() ||
                robotCommand.vel.y < 0.0 || robotCommand.vel.y > Constants::MAX_VEL_CMD()) {
            RTT_ERROR("Velocity not within acceptable range")
            return Status::Failure;
        }

        // Check if angle is in range
        if (robotCommand.angle.getAngle() < Constants::MIN_ANGLE() || robotCommand.angle.getAngle() > Constants::MAX_ANGLE()) {
            RTT_ERROR("Rotation angle not within acceptable range")
            return Status::Failure;
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
            return Status::Success;
        } else {
            return Status::Running;
        }
    }

    void GoToPos::onTerminate() noexcept { }

} // namespace rtt::ai::stp