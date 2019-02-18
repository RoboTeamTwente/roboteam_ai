//
// Created by baris on 15-2-19.
//


#include "Pause.h"

namespace rtt {
namespace pause {

bool Pause::pause = false;
std::mutex Pause::pauseLock;

bool Pause::getPause() {
    std::lock_guard<std::mutex> lock(pauseLock);
    return pause;
}
void Pause::haltRobots() {

    auto us = rtt::ai::World::get_world().us;
    for (auto robot : us) {
        roboteam_msgs::RobotCommand cmd;
        cmd.x_vel = 0;
        cmd.y_vel = 0;
        cmd.id = robot.id;
        IOManager.publishRobotCommand(cmd);
    }

}
void Pause::setPause(bool set) {
    std::lock_guard<std::mutex> lock(pauseLock);
    pause = set;

}
}
}