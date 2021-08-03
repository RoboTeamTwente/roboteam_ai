//
// Created by baris on 15-2-19.
//

#include "world/World.hpp"
#include "utilities/IOManager.h"

namespace rtt::ai {

bool Pause::pause = false;
std::mutex Pause::pauseLock;

bool Pause::getPause() {
    std::lock_guard<std::mutex> lock(pauseLock);
    return pause;
}
void Pause::haltRobots(rtt::world::World const* data) {
    auto us = data->getWorld()->getUs();
    std::vector<proto::RobotCommand> commands;
    for (const auto &robot : us) {
        proto::RobotCommand cmd;
        cmd.mutable_vel()->set_x(0);
        cmd.mutable_vel()->set_y(0);
        cmd.set_id(robot->getId());
        cmd.set_dribbler(0);
        cmd.set_use_angle(1);
        cmd.set_w(static_cast<float>(robot->getAngle()));
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