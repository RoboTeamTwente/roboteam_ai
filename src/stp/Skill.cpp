//
// Created by john on 3/2/20.
//

#include "include/roboteam_ai/stp/Skill.h"

#include <roboteam_utils/Print.h>

#include "control/ControlUtils.h"
#include "utilities/IOManager.h"
#include "utilities/Settings.h"
#include "world_new/World.hpp"

namespace rtt::ai::stp {

void Skill::rotateRobotCommand() noexcept {
    command.mutable_vel()->set_x(-command.vel().x());
    command.mutable_vel()->set_y(-command.vel().y());
    command.set_w(static_cast<float>(Angle(command.w() + M_PI)));
}

void Skill::publishRobotCommand() noexcept {
    limitRobotCommand();

    if (!SETTINGS.isLeft()) {
        rotateRobotCommand();
    }

    if (std::isnan(command.vel().x()) || std::isnan(command.vel().y())) {
        RTT_ERROR("x or y vel in command is NaN in skill" + std::string{getName()} + "!\nRobot: " + std::to_string(robot.value()->getId()))
    }

    if (command.id() == -1) {
        if (robot && robot.value()->getId() != -1) {
            command.set_id(robot.value()->getId());
            io::io.publishRobotCommand(command);  // We default to our robots being on the left if parameter is not set
        }
    } else {
        io::io.publishRobotCommand(command);  // We default to our robots being on the left if parameter is not set
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

    /* Limit the velocity when the robot has the ball
     * TODO: Test if it is necessary to limit the velocity when the robot has the ball
     * Might not be necessary because the robot is only allowed to move a small distance with the ball
     */
    double maxVel = 3.0; // Maximum velocity of the robot with ball TODO: TUNE

    if (robot->hasBall() && limitedVel.length() > maxVel) {
        // Clamp velocity
        limitedVel = control::ControlUtils::velocityLimiter(limitedVel, maxVel, 0.0, false);
    }

    command.mutable_vel()->set_x(limitedVel.x);
    command.mutable_vel()->set_y(limitedVel.y);
}

void Skill::limitAngularVel() noexcept {
    // Limit the angular velocity when the robot has the ball by setting the target angle in small steps
    // TODO: Might want to limit on the robot itself
    if (robot->hasBall() && command.use_angle()) {
        double angleRate = 0.1 * M_PI; // Angle increment per tick TODO: TUNE
        auto targetAngle = command.w();
        auto robotAngle = robot.value()->getAngle();

        // If the angle error is larger than the desired angle rate, the angle command is adjusted
        if (robotAngle.shortestAngleDiff(targetAngle) > angleRate) {
            // Direction of rotation is the shortest distance
            int direction = Angle(robotAngle).rotateDirection(targetAngle) ? 1 : -1;
            // Set the angle command to the current robot angle + the angle rate
            command.set_w(robotAngle + Angle(direction * angleRate));
        }
    }
}

void Skill::terminate() noexcept { onTerminate(); }

Status Skill::update(StpInfo const& info) noexcept {
    robot = info.getRobot();
    auto result = onUpdate(info);
    currentStatus = result;
    return result;
}

void Skill::initialize() noexcept { onInitialize(); }

[[nodiscard]] Status Skill::getStatus() const {
    return currentStatus;
}

}  // namespace rtt::ai::stp
