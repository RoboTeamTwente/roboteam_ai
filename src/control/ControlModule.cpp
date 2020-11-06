//
// Created by jaro on 15-10-20.
//

#include "control/ControlModule.h"
#include <roboteam_utils/Print.h>
#include "control/ControlUtils.h"
#include "utilities/IOManager.h"
#include "utilities/Settings.h"
#include "world/World.hpp"

namespace rtt::ai::control {

    proto::RobotCommand ControlModule::command;
    std::optional<::rtt::world::view::RobotView> ControlModule::robot;
    std::vector<proto::RobotCommand> ControlModule::robotCommands;

    void ControlModule::rotateRobotCommand() noexcept {
        command.mutable_vel()->set_x(-command.vel().x());
        command.mutable_vel()->set_y(-command.vel().y());
        command.set_w(static_cast<float>(Angle(command.w() + M_PI)));
    }

    void ControlModule::limitRobotCommand() noexcept {
        limitVel();
        limitAngularVel();
    }

    void ControlModule::limitVel() noexcept {
        auto limitedVel = Vector2(command.vel().x(), command.vel().y());

        limitedVel = control::ControlUtils::velocityLimiter(limitedVel);

        if (std::isnan(limitedVel.x) || std::isnan(limitedVel.y)) {
            RTT_ERROR("A certain Skill has produced a NaN error. \nRobot: " + std::to_string(robot.value()->getId()))
        }

        // Limit robot velocity when the robot has the ball
        if (robot->hasBall() && limitedVel.length() > stp::control_constants::MAX_VEL_WHEN_HAS_BALL) {
            // Clamp velocity
            limitedVel = control::ControlUtils::velocityLimiter(limitedVel, stp::control_constants::MAX_VEL_WHEN_HAS_BALL,
                                                                0.0, false);
        }

        command.mutable_vel()->set_x(static_cast<float>(limitedVel.x));
        command.mutable_vel()->set_y(static_cast<float>(limitedVel.y));
    }

    void ControlModule::limitAngularVel() noexcept {
        // Limit the angular velocity when the robot has the ball by setting the target angle in small steps
        // Might want to limit on the robot itself
        if (robot->hasBall() && command.use_angle()) {
            auto targetAngle = command.w();
            auto robotAngle = robot.value()->getAngle();

            // If the angle error is larger than the desired angle rate, the angle command is adjusted
            if (robotAngle.shortestAngleDiff(targetAngle) > stp::control_constants::ANGLE_RATE) {
                // Direction of rotation is the shortest distance
                int direction = Angle(robotAngle).rotateDirection(targetAngle) ? 1 : -1;
                // Set the angle command to the current robot angle + the angle rate
                command.set_w(static_cast<float>(robotAngle + Angle(direction * stp::control_constants::ANGLE_RATE)));
            }
        }
    }

    void ControlModule::publishRobotCommand(std::optional<::rtt::world::view::RobotView> robot_, const proto::RobotCommand& command_, const rtt::world::World *data) noexcept {
        robot = robot_;
        command = command_;

        // If we are not left, commands should be rotated (because we play as right)
        if (!SETTINGS.isLeft()) {
            rotateRobotCommand();
        }

        limitRobotCommand();

        std::cout << robot->get()->getId();
        robotCommands.emplace_back(command);

        io::io.publishRobotCommand(command, data);
    }
}  // namespace rtt::ai::stp