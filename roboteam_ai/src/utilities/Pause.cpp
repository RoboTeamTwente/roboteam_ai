//
// Created by baris on 15-2-19.
//


#include "Pause.h"
#include "../io/IOManager.h"

namespace rtt {
namespace ai {

bool Pause::pause = false;
std::mutex Pause::pauseLock;

bool Pause::getPause() {
    std::lock_guard<std::mutex> lock(pauseLock);
    return pause;
}
void Pause::haltRobots() {

    auto us = rtt::ai::World::get_world().us;
    for (const auto &robot : us) {
        roboteam_msgs::RobotCommand cmd;
        cmd.x_vel = 0;
        cmd.y_vel = 0;
        cmd.id = robot.id;
        cmd.dribbler = 0;
        cmd.w = robot.w;
        IOManager->publishRobotCommand(cmd);
    }

}
void Pause::setPause(bool set) {
    std::lock_guard<std::mutex> lock(pauseLock);
    pause = set;

}
Pause::Pause() = default;

}
}