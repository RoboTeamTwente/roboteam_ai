//
// Created by baris on 15-2-19.
//


#include "include/roboteam_ai/utilities/Pause.h"
#include "include/roboteam_ai/io/IOManager.h"
#include "include/roboteam_ai/world/Robot.h"

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
        roboteam_proto::RobotCommand cmd;
        cmd.mutable_vel()->set_x(0);
        cmd.mutable_vel()->set_y(0);
        cmd.set_id(robot->id);
        cmd.set_dribbler(0);
        cmd.set_use_angle(1);
        cmd.set_w(static_cast<float>(robot->angle));
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