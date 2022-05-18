//
// Created by jaro on 15-10-20.
//

#include "control/ControlModule.h"

#include <roboteam_utils/Print.h>

#include "control/ControlUtils.h"
#include "iostream"
#include "utilities/Constants.h"
#include "utilities/IOManager.h"
#include "utilities/Settings.h"
#include "world/World.hpp"

namespace rtt::ai::control {

void ControlModule::rotateRobotCommand(rtt::RobotCommand& command) {
    command.velocity.x = -command.velocity.x;
    command.velocity.y = -command.velocity.y;
    command.targetAngle += M_PI;
}

void ControlModule::limitRobotCommand(rtt::RobotCommand& command, std::optional<rtt::world::view::RobotView> robot) {
    // This was used to limit the velocity, but this is now done in pathtracking itself
    // limitVel(command, robot);
    limitAngularVel(command, robot);
}

void ControlModule::limitVel(rtt::RobotCommand& command, std::optional<rtt::world::view::RobotView> robot) {
    Vector2 limitedVelocity = command.velocity;

    limitedVelocity = control::ControlUtils::velocityLimiter(limitedVelocity);

    // TODO: Is this check really necessary? There is no stuff like division used...
    if (std::isnan(limitedVelocity.x) || std::isnan(limitedVelocity.y)) {
        RTT_ERROR("A certain Skill has produced a NaN error. \nRobot: " + std::to_string(robot.value()->getId()))
    }

    // Limit robot velocity when the robot has the ball
    if (robot->hasBall() && limitedVelocity.length() > stp::control_constants::MAX_VEL_WHEN_HAS_BALL) {
        // Clamp velocity
        limitedVelocity = control::ControlUtils::velocityLimiter(limitedVelocity, stp::control_constants::MAX_VEL_WHEN_HAS_BALL, 0.0, false);
    }

    command.velocity = limitedVelocity;
}

void ControlModule::limitAngularVel(rtt::RobotCommand& command, std::optional<rtt::world::view::RobotView> robot) {
    // Limit the angular velocity when the robot has the ball by setting the target angle in small steps
    // Might want to limit on the robot itself
    if (robot->hasBall() && !command.useAngularVelocity) {
        auto targetAngle = command.targetAngle;
        // TODO: Why use optional robotView if we never check for the case where it does not contain one?
        auto robotAngle = robot.value()->getAngle();

        // If the angle error is larger than the desired angle rate, the angle command is adjusted
        if (robotAngle.shortestAngleDiff(targetAngle) > stp::control_constants::ANGLE_RATE) {
            // Direction of rotation is the shortest distance
            int direction = Angle(robotAngle).rotateDirection(targetAngle) ? 1 : -1;
            // Set the angle command to the current robot angle + the angle rate
            command.targetAngle = robotAngle + Angle(direction * stp::control_constants::ANGLE_RATE);
        }
    }
    else if (robot->hasBall()){
        // TODO: Tune this value
        constexpr double maxAngularVelWithBall = 0.5;
        command.targetAngularVelocity = std::clamp(command.targetAngularVelocity, -maxAngularVelWithBall, maxAngularVelWithBall);
    }
}

void ControlModule::addRobotCommand(std::optional<::rtt::world::view::RobotView> robot, const rtt::RobotCommand& command, const rtt::world::World* data) noexcept {
    rtt::RobotCommand robot_command = command;  // TODO: Why make a copy of the command? It will be copied anyway when we put it in the vector

    if (robot && robot->get()) {
        Angle target = command.targetAngle;
        interface::Input::drawData(interface::Visual::PATHFINDING, {robot->get()->getPos(), robot->get()->getPos() + Vector2(target)}, Qt::red, robot->get()->getId(),
                                   interface::Drawing::LINES_CONNECTED);
    }

    if (robot) limitRobotCommand(robot_command, robot);

    // if we are in simulation; adjust w() to be angular velocity)
    if (SETTINGS.getRobotHubMode() == Settings::RobotHubMode::SIMULATOR) {
        simulator_angular_control(robot, robot_command);
    }

    bool useAngVelOnField = true;
    if (useAngVelOnField) setAngularVelocity(robot, robot_command);

    // If we are not left, commands should be rotated (because we play as right)
    if (!SETTINGS.isLeft()) {
        rotateRobotCommand(robot_command);
    }

    // Only add commands with a robotID that is not in the vector yet
    // This mutex is required because robotCommands is accessed from both the main thread and joystick thread
    std::lock_guard<std::mutex> guard(robotCommandsMutex);
    if (robot_command.id >= 0 && robot_command.id < 16) {
        robotCommands.emplace_back(robot_command);
    }
}

void ControlModule::simulator_angular_control(const std::optional<::rtt::world::view::RobotView>& robot, rtt::RobotCommand& robot_command) {
    double ang_velocity_out = 0.0;  // in case there is no robot visible, we just adjust the command to not have any angular velocity
    if (robot) {
        Angle current_angle = robot->get()->getAngle();

        Angle target_angle(robot_command.targetAngle);
        // get relevant PID controller
        if (simulatorAnglePIDmap.contains(robot->get()->getId())) {
            auto extrapolated_angle = current_angle + Angle(simulatorAnglePIDmap.at(robot->get()->getId()).getPreviousOutput() / Constants::STP_TICK_RATE());
            ang_velocity_out = simulatorAnglePIDmap.at(robot->get()->getId()).getOutput(target_angle, extrapolated_angle);
        } else {
            // initialize PID controller for robot
            // below tuning only works ish for erforce, is completely useless in grsim
            double P = 4.0;
            double I = 0.0;
            double D = 0.01;
            double max_ang_vel = 5.0;  // rad/s
            double dt = 1. / double(Constants::STP_TICK_RATE());

            AnglePID pid(P, I, D, max_ang_vel, dt);
            ang_velocity_out = pid.getOutput(target_angle, current_angle);
            simulatorAnglePIDmap.insert({robot->get()->getId(), pid});
        }
    }
    robot_command.useAngularVelocity = true;
    ang_velocity_out = std::clamp(ang_velocity_out, -8.0 * M_PI, 8.0 * M_PI);
    robot_command.targetAngularVelocity = static_cast<float>(ang_velocity_out);
}

void ControlModule::sendAllCommands() {
    // TODO: check for double commands (But do we really have to do that?)

    // This mutex is required because robotCommands is accessed from both the main thread and joystick thread
    std::lock_guard<std::mutex> guard(robotCommandsMutex);
    io::io.publishAllRobotCommands(robotCommands);  // When vector has all commands, send in one go
    robotCommands.clear();
}
void ControlModule::setAngularVelocity(const std::optional<::rtt::world::view::RobotView>& robot, RobotCommand& robot_command) {
    double ang_velocity_out = 0.0;  // in case there is no robot visible, we just adjust the command to not have any angular velocity
    Angle current_angle = robot->get()->getAngle();

    Angle target_angle = robot_command.targetAngle;

    // get relevant PID controller
    if (angularVelPIDMap.contains(robot->get()->getId())) {
        ang_velocity_out = angularVelPIDMap.at(robot->get()->getId()).getOutput(target_angle, current_angle);
    } else {
        // initialize PID controller for robot
        // TODO: tune these values for on the field
        double P = 4.0;
        double I = 0.0;
        double D = 0.01;
        double max_ang_vel = 5.0;  // rad/s
        double dt = 1. / double(Constants::STP_TICK_RATE());

        AnglePID pid(P, I, D, max_ang_vel, dt);
        ang_velocity_out = pid.getOutput(target_angle, current_angle);
        angularVelPIDMap.insert({robot->get()->getId(), pid});
    }
    robot_command.useAngularVelocity = true;
    ang_velocity_out = std::clamp(ang_velocity_out, -8.0 * M_PI, 8.0 * M_PI);
    robot_command.targetAngularVelocity = static_cast<float>(ang_velocity_out);
    }
}  // namespace rtt::ai::control