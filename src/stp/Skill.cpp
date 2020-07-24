//
// Created by john on 3/2/20.
//

#include "stp/Skill.h"

#include <roboteam_utils/Print.h>

#include "control/ControlUtils.h"
#include "utilities/IOManager.h"
#include "utilities/Settings.h"
#include "world/World.hpp"

namespace rtt::ai::stp {

void Skill::rotateRobotCommand() noexcept {
    command.mutable_vel()->set_x(-command.vel().x());
    command.mutable_vel()->set_y(-command.vel().y());
    command.set_w(static_cast<float>(Angle(command.w() + M_PI)));
}

void Skill::publishRobotCommand(world::World const* data) noexcept {
    limitRobotCommand();

    // If we are not left, commands should be rotated (because we play as right)
    if (!SETTINGS.isLeft()) {
        rotateRobotCommand();
    }

    if (std::isnan(command.vel().x()) || std::isnan(command.vel().y())) {
        RTT_ERROR("x or y vel in command is NaN in skill" + std::string{getName()} + "!\nRobot: " + std::to_string(robot.value()->getId()))
    }

    if (command.id() == -1) {
        if (robot && robot.value()->getId() != -1) {
            command.set_id(robot.value()->getId());
            io::io.publishRobotCommand(command, data);
        }
    } else {
        io::io.publishRobotCommand(command, data);
    }

    // refresh the robot command after it has been sent
    refreshRobotCommand();
}

void Skill::refreshRobotCommand() noexcept {
    proto::RobotCommand emptyCmd;
    emptyCmd.set_use_angle(true);
    emptyCmd.set_id(robot ? robot.value()->getId() : -1);
    emptyCmd.set_geneva_state(0);
    command = emptyCmd;
}

void Skill::limitRobotCommand() noexcept {
    limitVel();
    limitAngularVel();
}

void Skill::limitVel() noexcept {
    auto limitedVel = Vector2(command.vel().x(), command.vel().y());

    limitedVel = control::ControlUtils::velocityLimiter(limitedVel);

    if (std::isnan(limitedVel.x) || std::isnan(limitedVel.y)) {
        RTT_ERROR("Robot will have NAN: " + std::string{getName()} + "!\nrobot: " + std::to_string(robot.value()->getId()))
    }

    // Limit robot velocity when the robot has the ball
    if (robot->hasBall() && limitedVel.length() > control_constants::MAX_VEL_WHEN_HAS_BALL) {
        // Clamp velocity
        limitedVel = control::ControlUtils::velocityLimiter(limitedVel, control_constants::MAX_VEL_WHEN_HAS_BALL, 0.0, false);
    }

    command.mutable_vel()->set_x(static_cast<float>(limitedVel.x));
    command.mutable_vel()->set_y(static_cast<float>(limitedVel.y));
}

void Skill::limitAngularVel() noexcept {
    // Limit the angular velocity when the robot has the ball by setting the target angle in small steps
    // TODO: Might want to limit on the robot itself
    if (robot->hasBall() && command.use_angle()) {
        auto targetAngle = command.w();
        auto robotAngle = robot.value()->getAngle();

        // If the angle error is larger than the desired angle rate, the angle command is adjusted
        if (robotAngle.shortestAngleDiff(targetAngle) > control_constants::ANGLE_RATE) {
            // Direction of rotation is the shortest distance
            int direction = Angle(robotAngle).rotateDirection(targetAngle) ? 1 : -1;
            // Set the angle command to the current robot angle + the angle rate
            command.set_w(static_cast<float>(robotAngle + Angle(direction * control_constants::ANGLE_RATE)));
        }
    }
}

void Skill::terminate() noexcept {}

Status Skill::update(StpInfo const& info) noexcept {
    robot = info.getRobot();
    auto result = onUpdate(info);
    currentStatus = result;
    return result;
}

void Skill::initialize() noexcept {}

[[nodiscard]] Status Skill::getStatus() const { return currentStatus; }

}  // namespace rtt::ai::stp
