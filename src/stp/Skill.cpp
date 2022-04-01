//
// Created by john on 3/2/20.
//

#include "stp/Skill.h"

#include "control/ControlModule.h"
#include "world/World.hpp"

namespace rtt::ai::stp {

void Skill::forwardRobotCommand(world::World const* data) noexcept {
    // The command gets passed to the ControlModule here, for checking and limiting
    control::ControlModule::addRobotCommand(robot, command, data);

    // refresh the robot command after it has been sent
    refreshRobotCommand();
}

void Skill::refreshRobotCommand() noexcept {
    rtt::RobotCommand emptyCmd = {};

    emptyCmd.id = this->robot ? this->robot.value()->getId() : -1;

    this->command = emptyCmd;
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
