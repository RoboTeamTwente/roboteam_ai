//
// Created by baris on 15-2-19.
//

#include <roboteam_utils/RobotCommands.hpp>

#include "utilities/IOManager.h"
#include "world/World.hpp"

namespace rtt::ai {

bool Pause::pause = false;
std::mutex Pause::pauseLock;

bool Pause::getPause() {
    std::lock_guard<std::mutex> lock(pauseLock);
    return pause;
}
void Pause::haltRobots(rtt::world::World const* data) {
    auto us = data->getWorld()->getUs();
    std::vector<rtt::RobotCommand> commands;
    for (const auto& robot : us) {
        rtt::RobotCommand cmd = {};
        cmd.id = robot->getId();
        cmd.velocity.x = 0;
        cmd.velocity.y = 0;
        cmd.useAngularVelocity = false;
        cmd.targetAngle = robot->getAngle();
        commands.push_back(std::move(cmd));
    }
    io::io.publishAllRobotCommands(commands);
}
void Pause::setPause(bool set) {
    std::lock_guard<std::mutex> lock(pauseLock);
    pause = set;
}
Pause::Pause() {}

}  // namespace rtt::ai