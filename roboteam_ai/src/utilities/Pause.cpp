//
// Created by baris on 15-2-19.
//


#include "Pause.h"
#include "../io/IOManager.h"
#include "../world/Robot.h"

namespace rtt {
namespace ai {

bool Pause::pause = false;
std::mutex Pause::pauseLock;

bool Pause::getPause() {
    std::lock_guard<std::mutex> lock(pauseLock);
    return pause;
}
void Pause::haltRobots() {

    auto us = world::world->getUs();
    for (const auto &robot : us) {
        roboteam_msgs::RobotCommand cmd;
        cmd.x_vel = 0;
        cmd.y_vel = 0;
        cmd.id = robot->id;
        cmd.dribbler = 0;
        cmd.use_angle = 1;
        cmd.w = static_cast<float>(robot->angle);
        IOManager->publishRobotCommand(cmd);
    }

}
void Pause::setPause(bool set) {
    std::lock_guard<std::mutex> lock(pauseLock);
    pause = set;

}
Pause::Pause() {
    io::IOManager ioManager;
    IOManager = std::make_shared<io::IOManager>(ioManager);
}

}
}